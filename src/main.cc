#include "data.h"
#include "loadKittiData.h"
#include <iomanip>

#include <sym/pose3.h>
#include <sym/util/epsilon.h>
#include <symforce/opt/factor.h>
#include <symforce/opt/optimizer.h>
#include <symforce/slam/imu_preintegration/imu_factor.h>
#include <symforce/slam/imu_preintegration/imu_preintegrator.h>
#include <symforce/slam/imu_preintegration/preintegrated_imu_measurements.h>

Eigen::Vector3d GetAccelBiasEstimate(double time) {
  return Eigen::Vector3d(0.0, 0.0, 0.0);
}

Eigen::Vector3d GetGyroBiasEstimate(double time) {
  return Eigen::Vector3d(0.00, 0.00, 0.00);
}

void AppendOtherFactors(std::vector<sym::Factord> &factors);

std::vector<ImuMeasurement>
GetMeasurementsBetween(double start_time, double end_time,
                       const std::vector<ImuMeasurement> &imu_measurements) {
  std::vector<ImuMeasurement> result;
  for (const auto &meas : imu_measurements) {
    if (meas.time >= start_time && meas.time <= end_time) {
      result.push_back(meas);
    }
  }
  return result;
}

std::pair<sym::Valuesd, std::vector<sym::Factord>>
buildValuesAndFactors(const std::vector<ImuMeasurement> &imu_measurements,
                      const std::vector<GpsMeasurement> &gps_measurements) {
  // Build Values
  sym::Valuesd values;
  std::vector<sym::Factord> factors;
  int first_gps_pose = 0;
  int gps_skip = 10;
  const Eigen::Vector3d accel_cov = Eigen::Vector3d::Constant(1e-5);
  const Eigen::Vector3d gyro_cov = Eigen::Vector3d::Constant(1e-5);

  // gravity should point towards the direction of accelerometer
  // ENU
  values.Set({Var::GRAVITY}, Eigen::Vector3d(0.0, 0.0, -9.81));
  values.Set({Var::EPSILON}, sym::kDefaultEpsilond);

  for (size_t i = first_gps_pose; i < gps_measurements.size() - 1; i++) {

    if (i == first_gps_pose) {
      // TODO: create initial estimate and prior
    }

    const std::vector<ImuMeasurement> selected_imu_measurements =
        GetMeasurementsBetween(gps_measurements[i - 1], gps_measurements[i],
                               imu_measurements);
    sym::ImuPreintegrator<double> integrator(accel_bias, gyro_bias);

    // Integrate the measurrements together
    for (const auto &meas : selected_imu_measurements) {
      integrator.IntegrateMeasurement(meas.accelerometer, meas.gyroscope,
                                      accel_cov, gyro_cov, meas.dt);
    }
    // add single ImuFactor from the pre-integrated IMU measurrements
    factors.push_back(sym::ImuFactor<double>(integrator)
                          .Factor({{Var::POSE, i},
                                   {Var::VELOCITY, i},
                                   {Var::POSE, i + 1},
                                   {Var::VELOCITY, i + 1},
                                   {Var::ACCEL_BIAS, i},
                                   {Var::GYRO_BIAS, i},
                                   {Var::GRAVITY},
                                   {Var::EPSILON}}));

    // add between factors for bias
    // TODO: create these factors in generate.py
    factors.push_back(accel_bias_factor);
    factors.push_back(gyro_bias_factor);

    if (i % gps_skip == 0) {
      // add relative factor for gps
      // PriorFactorPose3
      // ssqrt for rot should be zeros
      // prior - value
      factors.push_back(sym::Factord::Hessian(
          sym::PriorFactorPose3<double>,
          {sym::Keys::POSE.WithSuper(i + 1), sym::Keys::POSE.WithSuper(i),
           sym::Keys::SQRT_INFO, sym::Keys::epsilon}));
    }

    values.Set({Var::POSE, i},
               sym::Pose3d(sym::Rot3d(), gps_measurements[i].position));
    // use IMU prediction here
    values.Set({Var::VELOCITY, i}, Eigen::Vector3d::Zero());

    values.Set({Var::ACCEL_BIAS, i}, GetAccelBiasEstimate(i));
    values.Set({Var::GYRO_BIAS, i}, GetGyroBiasEstimate(i));
  }

  return {values, factors};
}

int main() {
  KittiCalibration kitti_calibration;
  std::vector<ImuMeasurement> imu_measurements;
  std::vector<GpsMeasurement> gps_measurements;

  loadKittiData(kitti_calibration, imu_measurements, gps_measurements);

  visualizeData(imu_measurements, gps_measurements, 2);

  auto [values, factors] =
      buildValuesAndFactors(imu_measurements, gps_measurements);

  sym::Optimizerd optimizer(sym::DefaultOptimizerParams(), factors);
  optimizer.Optimize(values);

  visualizeTrajectory(values, gps_measurements);

  return 0;
}