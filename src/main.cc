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
#include "data.h"
#include "loadKittiData.h"

Eigen::Vector3d GetAccelBiasEstimate(double time) { return Eigen::Vector3d(0.0, 0.0, 0.0); }

Eigen::Vector3d GetGyroBiasEstimate(double time) { return Eigen::Vector3d(0.00, 0.00, 0.00); }

std::vector<ImuMeasurement> GetMeasurementsBetween(double start_time, double end_time, const std::vector<ImuMeasurement> &imu_measurements) {
  std::vector<ImuMeasurement> result;
  for (const auto &meas : imu_measurements) {
    if (meas.time >= start_time && meas.time <= end_time) {
      result.push_back(meas);
    }
  }
  return result;
}

std::pair<sym::Valuesd, std::vector<sym::Factord>> buildValuesAndFactors(const std::vector<ImuMeasurement> &imu_measurements, const std::vector<GpsMeasurement> &gps_measurements,
                                                                         KittiCalibration kitti_calibration) {
  sym::Valuesd values;
  std::vector<sym::Factord> factors;

  Eigen::Vector3d accel_cov = Eigen::Vector3d::Constant(std::pow(kitti_calibration.accelerometer_sigma, 2));
  Eigen::Vector3d gyro_cov = Eigen::Vector3d::Constant(std::pow(kitti_calibration.gyroscope_sigma, 2));

  // CONSTANTS
  values.Set({sym::Keys::GRAVITY}, Eigen::Vector3d(0.0, 0.0, -9.81));
  values.Set({sym::Keys::EPSILON}, sym::kDefaultEpsilond);

  Eigen::Matrix<double, 6, 6> sqrt_info = Eigen::Matrix<double, 6, 6>::Zero();
  // GTSAM uses the precision matrix (information matrix) while symforce uses the square root of this
  sqrt_info.diagonal().segment<3>(3) = Eigen::Vector3d::Constant(1.0 / std::sqrt(0.07)).array().sqrt();

  values.Set({sym::Keys::SQRT_INFO}, sqrt_info);

  values.Set(sym::Keys::ACCEL_BIAS_DIAG_SQRT_INFO, Eigen::Vector3d::Constant(1 / kitti_calibration.accelerometer_bias_sigma));
  values.Set(sym::Keys::GYRO_BIAS_DIAG_SQRT_INFO, Eigen::Vector3d::Constant(1 / kitti_calibration.gyroscope_bias_sigma));

  // const sym::Matrix66d sqrt_info = sym::Vector6d::Constant(1 / sigma).asDiagonal();

  // const int NUM_FACTORS = gps_measurements.size() - 1;
  const int NUM_FACTORS = 15;

  // NOTE: we run this on the basis that a factor is created in the loop between i and i+1, not between i and i-1

  sym::Pose3d current_pose = sym::Pose3d();
  Eigen::Vector3d current_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d current_accel_bias_estimate = Eigen::Vector3d::Zero();
  Eigen::Vector3d current_gyro_bias_estimate = Eigen::Vector3d::Zero();

  /// ITERATE one pose at a time and optimize
  for (size_t i = 0; i < NUM_FACTORS; i++) {
    values.Set(sym::Keys::ACCEL_BIAS.WithSuper(i), current_accel_bias_estimate);
    values.Set(sym::Keys::VELOCITY.WithSuper(i), current_velocity);
    values.Set(sym::Keys::POSE.WithSuper(i), current_pose);
    values.Set(sym::Keys::GYRO_BIAS.WithSuper(i), current_gyro_bias_estimate);
    values.Set(sym::Keys::MEASURED_POSE.WithSuper(i), sym::Pose3d(sym::Rot3d(), gps_measurements[i].position));
    if (i == 0) {
      continue;
    }

    if (i > NUM_FACTORS - 1) {
      continue;
    }

    // IMU factor
    double sum_dt = 0.0;
    const std::vector<ImuMeasurement> selected_imu_measurements = GetMeasurementsBetween(gps_measurements[i - 1].time, gps_measurements[i].time, imu_measurements);
    sym::ImuPreintegrator<double> integrator(current_accel_bias_estimate, current_gyro_bias_estimate);
    for (const auto &meas : selected_imu_measurements) {
      sum_dt += meas.dt;
      integrator.IntegrateMeasurement(meas.accelerometer, meas.gyroscope, accel_cov, gyro_cov, meas.dt);
    }
    factors.push_back(createImuFactor(i, integrator));
    if (i == 1) {
      values.Set(sym::Keys::TIME_DELTA.WithSuper(i), 0.0);

    } else {
      std::cout << "sum dt " << sum_dt << std::endl;
      values.Set(sym::Keys::TIME_DELTA.WithSuper(i), sum_dt);
    }

    // Bias factors
    factors.push_back(createAccelBiasFactor(i));
    factors.push_back(createGyroBiasFactor(i));
    factors.push_back(createPoseFactor(i));

    std::cout << "Optimizing.." << std::endl;
    sym::optimizer_params_t optimizer_params = sym::DefaultOptimizerParams();
    // optimizer_params.debug_checks = true;
    // optimizer_params.verbose = true;
    // optimizer_params.debug_stats = true;
    // optimizer_params.check_derivatives = true;
    sym::Optimizerd optimizer(optimizer_params, factors);
    // std::cout << values;
    optimizer.Optimize(values);

    sym::Pose3d current_pose = values.At<sym::Pose3d>(sym::Keys::POSE.WithSuper(i));
    current_velocity = values.At<Eigen::Vector3d>(sym::Keys::VELOCITY.WithSuper(i));
    current_accel_bias_estimate = values.At<Eigen::Vector3d>(sym::Keys::ACCEL_BIAS.WithSuper(i));
    current_gyro_bias_estimate = values.At<Eigen::Vector3d>(sym::Keys::GYRO_BIAS.WithSuper(i));

    std::cout << "Optimized pose at " << i << " is " << current_pose.Position().transpose() << ", measured pose is " << gps_measurements[i].position.transpose() << std::endl;

    std::cout << "Velocity at " << i << ": " << current_velocity.transpose() << std::endl;
    std::cout << "Accelerometer Bias at " << i << ": " << current_accel_bias_estimate.transpose() << std::endl;
    std::cout << "Gyroscope Bias at " << i << ": " << current_gyro_bias_estimate.transpose() << std::endl;
    current_accel_bias_estimate = Eigen::Vector3d::Zero();
    current_gyro_bias_estimate = Eigen::Vector3d::Zero();
    current_velocity = Eigen::Vector3d::Zero();
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

  // std::cout << "factors: " << factors;

  // std::cout << "Optimizing.." << std::endl;
  // sym::optimizer_params_t optimizer_params = sym::DefaultOptimizerParams();
  // optimizer_params.debug_checks = true;
  // optimizer_params.verbose = true;
  // optimizer_params.debug_stats = true;
  // // optimizer_params.check_derivatives = false;
  // // optimizer_params.include_jacobians = false;

  // sym::Optimizerd optimizer(optimizer_params, factors);
  // optimizer.Optimize(values);
  // std::cout << "values: " << values;

  visualizeTrajectory(values, gps_measurements, 15);

  return 0;
}