#include <stdio.h>
#include <sym/pose3.h>
#include <sym/util/epsilon.h>
#include <symforce/opt/factor.h>
#include <symforce/opt/optimizer.h>
#include <symforce/slam/imu_preintegration/imu_factor.h>
#include <symforce/slam/imu_preintegration/imu_preintegrator.h>
#include <symforce/slam/imu_preintegration/preintegrated_imu_measurements.h>

#include <iomanip>

#include "../gen/keys.h"
#include "createFactors.h"
#include "loadKittiData.h"
#include "visualize.h"

const int GPS_SKIP = 10;

std::vector<ImuMeasurement> GetMeasurementsBetween(double start_time, double end_time, const std::vector<ImuMeasurement> &imu_measurements) {
  std::vector<ImuMeasurement> result;
  for (const auto &meas : imu_measurements) {
    if (meas.time >= start_time && meas.time <= end_time) {
      result.push_back(meas);
    }
  }
  return result;
}

void PrintState(int state_index, const sym::Valuesd &values, const std::vector<GpsMeasurement> &gps_measurements) {
  std::cout << "State at #" << state_index + 1 << "\n";

  // Pose
  auto current_pose = values.At<sym::Pose3d>(sym::Keys::POSE.WithSuper(state_index));
  std::cout << "Pose:\n";
  std::cout << "R: [\n";
  std::cout << "  " << current_pose.Rotation().ToRotationMatrix().row(0) << ";\n";
  std::cout << "  " << current_pose.Rotation().ToRotationMatrix().row(1) << ";\n";
  std::cout << "  " << current_pose.Rotation().ToRotationMatrix().row(2) << "\n";
  std::cout << "]\n";
  std::cout << "t: " << current_pose.Position().transpose() << "\n";

  // Velocity
  auto current_velocity = values.At<Eigen::Vector3d>(sym::Keys::VELOCITY.WithSuper(state_index));
  std::cout << "Velocity:\n" << current_velocity.transpose() << "\n";

  // Biases
  auto accel_bias = values.At<Eigen::Vector3d>(sym::Keys::ACCEL_BIAS.WithSuper(state_index));
  auto gyro_bias = values.At<Eigen::Vector3d>(sym::Keys::GYRO_BIAS.WithSuper(state_index));
  std::cout << "Bias:\n";
  std::cout << "acc = " << accel_bias.transpose() << " ";
  std::cout << "gyro = " << gyro_bias.transpose() << "\n";

  // Sqrt Information
  auto accel_sqrt_info = values.At<Eigen::Vector3d>(sym::Keys::ACCEL_BIAS_DIAG_SQRT_INFO.WithSuper(state_index));
  auto gyro_sqrt_info = values.At<Eigen::Vector3d>(sym::Keys::GYRO_BIAS_DIAG_SQRT_INFO.WithSuper(state_index));
  std::cout << "AccelerometerBias sqrt info at " << state_index << ": " << accel_sqrt_info.transpose() << "\n";
  std::cout << "GyroscopeBias sqrt info at " << state_index << ": " << gyro_sqrt_info.transpose() << "\n";

  // Pose comparison
  std::cout << "Optimized pose at " << state_index << " is " << current_pose.Position().transpose();
  std::cout << ", gps measured pose is " << gps_measurements[state_index].position.transpose() << std::endl;
}

void initializeValuesKITTI(sym::Valuesd &values, const KittiCalibration &kitti_calibration) {
  values.Set({sym::Keys::GRAVITY}, Eigen::Vector3d(0.0, 0.0, -9.81));
  values.Set({sym::Keys::EPSILON}, sym::kDefaultEpsilond);

  // GTSAM uses the precision matrix (information matrix) while symforce uses the square root of this
  double precision_sqrt = 1.0 / std::sqrt(0.07);
  sym::Matrix66d meas_pose_sqrt_info = (sym::Vector6d() << 0, 0, 0, precision_sqrt, precision_sqrt, precision_sqrt).finished().asDiagonal();
  values.Set({sym::Keys::MEAS_POSE_SQRT_INFO}, meas_pose_sqrt_info);

  sym::Matrix66d pose_prior_sqrt_info = (sym::Vector6d() << 0, 0, 0, 1.0, 1.0, 1.0).finished().asDiagonal();
  values.Set({sym::Keys::POSE_PRIOR_SQRT_INFO}, pose_prior_sqrt_info);

  values.Set(sym::Keys::ACCEL_BIAS_PRIOR_SQRT_INFO, Eigen::Matrix3d::Identity() * (1 / 0.1));
  values.Set(sym::Keys::GYRO_BIAS_PRIOR_SQRT_INFO, Eigen::Matrix3d::Identity() * (1 / 5.00e-05));
  values.Set(sym::Keys::VELOCITY_PRIOR_SQRT_INFO, Eigen::Matrix3d::Identity() * (1 / 1000));

  values.Set(sym::Keys::ACCEL_COV, Eigen::Vector3d::Constant(std::pow(kitti_calibration.accelerometer_sigma, 2)));
  values.Set(sym::Keys::GYRO_COV, Eigen::Vector3d::Constant(std::pow(kitti_calibration.gyroscope_sigma, 2)));
}

std::pair<sym::Valuesd, std::vector<sym::Factord>> buildValuesAndFactors(const std::vector<ImuMeasurement> &imu_measurements, const std::vector<GpsMeasurement> &gps_measurements,
                                                                         KittiCalibration kitti_calibration) {
  sym::Valuesd values;
  std::vector<sym::Factord> factors;
  sym::optimizer_params_t optimizer_params = sym::DefaultOptimizerParams();

  initializeValuesKITTI(values, kitti_calibration);

  sym::Pose3d current_pose = sym::Pose3d();
  Eigen::Vector3d current_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d current_accel_bias_estimate = Eigen::Vector3d::Zero();
  Eigen::Vector3d current_gyro_bias_estimate = Eigen::Vector3d::Zero();

  /// ITERATE one pose at a time and optimize
  const int first_pose = 1;
  for (size_t i = 0; i < gps_measurements.size(); i++) {
    values.Set(sym::Keys::POSE.WithSuper(i), current_pose);
    values.Set(sym::Keys::VELOCITY.WithSuper(i), current_velocity);
    values.Set(sym::Keys::ACCEL_BIAS.WithSuper(i), current_accel_bias_estimate);
    values.Set(sym::Keys::GYRO_BIAS.WithSuper(i), current_gyro_bias_estimate);

    if (i < first_pose) {
      continue;
    }

    if (i == first_pose) {
      values.Set(sym::Keys::POSE.WithSuper(i - 1), sym::Pose3d(sym::Rot3d(), gps_measurements[i].position));
      factors.push_back(createAccelBiasPriorFactor(i));
      factors.push_back(createGyroBiasPriorFactor(i));
      factors.push_back(createPriorPoseFactor(i));
      factors.push_back(createVelocityPriorFactor(i));

      values.Set(sym::Keys::TIME_DELTA.WithSuper(i), 0.0);

    } else {
      // IMU Preintegration
      double sum_dt = 0.0;
      const std::vector<ImuMeasurement> selected_imu_measurements = GetMeasurementsBetween(gps_measurements[i - 1].time, gps_measurements[i].time, imu_measurements);
      sym::ImuPreintegrator<double> integrator(current_accel_bias_estimate, current_gyro_bias_estimate);
      for (const auto &meas : selected_imu_measurements) {
        sum_dt += meas.dt;
        integrator.IntegrateMeasurement(meas.accelerometer, meas.gyroscope, values.At<Eigen::Vector3d>(sym::Keys::ACCEL_COV), values.At<Eigen::Vector3d>(sym::Keys::GYRO_COV), meas.dt);
      }

      values.Set(sym::Keys::TIME_DELTA.WithSuper(i), sum_dt);

      int sqrt_num_measurements = std::sqrt(selected_imu_measurements.size());

      values.Set(sym::Keys::ACCEL_BIAS_DIAG_SQRT_INFO.WithSuper(i), Eigen::Vector3d::Constant(1 / (sqrt_num_measurements * kitti_calibration.accelerometer_bias_sigma)));
      values.Set(sym::Keys::GYRO_BIAS_DIAG_SQRT_INFO.WithSuper(i), Eigen::Vector3d::Constant(1 / (sqrt_num_measurements * kitti_calibration.gyroscope_bias_sigma)));

      // Add factors to graph
      factors.push_back(createImuFactor(i, integrator));
      factors.push_back(createAccelBiasFactor(i));
      factors.push_back(createGyroBiasFactor(i));
      if ((i % GPS_SKIP) == 0 && i > 1) {
        values.Set(sym::Keys::MEASURED_POSE.WithSuper(i), sym::Pose3d(current_pose.Rotation(), gps_measurements[i].position));
        factors.push_back(createMeasuredPoseFactor(i));
      }
    }

    if (i > 1) {
      sym::Optimizerd optimizer(optimizer_params, factors);
      optimizer.Optimize(values);
      // Update pose,velocity and biases for next step to be the optimized values
      current_pose = values.At<sym::Pose3d>(sym::Keys::POSE.WithSuper(i));
      current_velocity = values.At<Eigen::Vector3d>(sym::Keys::VELOCITY.WithSuper(i));
      current_accel_bias_estimate = values.At<Eigen::Vector3d>(sym::Keys::ACCEL_BIAS.WithSuper(i));
      current_gyro_bias_estimate = values.At<Eigen::Vector3d>(sym::Keys::GYRO_BIAS.WithSuper(i));

      if (i == gps_measurements.size() - 1) {
        PrintState(i, values, gps_measurements);
      }
    }
  }

  return {values, factors};
}

int main() {
  KittiCalibration kitti_calibration;
  std::vector<ImuMeasurement> imu_measurements;
  std::vector<GpsMeasurement> gps_measurements;

  loadKittiData(kitti_calibration, imu_measurements, gps_measurements);
  visualizeData(imu_measurements, gps_measurements, 12);

  auto [values, factors] = buildValuesAndFactors(imu_measurements, gps_measurements, kitti_calibration);

  visualizeTrajectory(values, gps_measurements, gps_measurements.size());

  return 0;
}