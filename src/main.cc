#include "data.h"

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
    if (meas.timestamp >= start_time && meas.timestamp <= end_time) {
      result.push_back(meas);
    }
  }
  return result;
}

std::vector<double>
GetKeyFrameTimes(const std::vector<GpsMeasurement> &gps_measurements) {
  std::vector<double> key_frame_times;
  for (const auto &gps : gps_measurements) {
    key_frame_times.push_back(gps.time);
  }
  return key_frame_times;
}

std::vector<sym::Factord>
createIMUfactors(const std::vector<double> &key_frame_times,
                 const std::vector<ImuMeasurement> &imu_measurements) {
  std::vector<sym::Factord> factors;

  const Eigen::Vector3d accel_cov = Eigen::Vector3d::Constant(1e-5);
  const Eigen::Vector3d gyro_cov = Eigen::Vector3d::Constant(1e-5);

  // For each gps factor
  for (int i = 0; i < static_cast<int>(key_frame_times.size()) - 1; i++) {
    const double start_time = key_frame_times[i];
    const double end_time = key_frame_times[i + 1];

    // get all 10 IMU measurements from this time period
    const std::vector<ImuMeasurement> measurements =
        GetMeasurementsBetween(start_time, end_time, imu_measurements);

    sym::ImuPreintegrator<double> integrator(GetAccelBiasEstimate(start_time),
                                             GetGyroBiasEstimate(start_time));

    // Integrate the measurrements together
    for (size_t i = 0; i < measurements.size() - 1; ++i) {
      const ImuMeasurement &meas = measurements[i];
      double duration = measurements[i + 1].timestamp - meas.timestamp;
      integrator.IntegrateMeasurement(meas.acceleration, meas.angular_velocity,
                                      accel_cov, gyro_cov, duration);
    }
    // add single ImuFactor from the 10 pre-integrated IMU measurrements
    factors.push_back(sym::ImuFactor<double>(integrator)
                          .Factor({{Var::POSE, i},
                                   {Var::VELOCITY, i},
                                   {Var::POSE, i + 1},
                                   {Var::VELOCITY, i + 1},
                                   {Var::ACCEL_BIAS, i},
                                   {Var::GYRO_BIAS, i},
                                   {Var::GRAVITY},
                                   {Var::EPSILON}}));
  }
  return factors;
}

sym::Valuesd optimizeImu(std::vector<sym::Factord> &factors,
                         const std::vector<double> &key_frame_times,
                         const std::vector<GpsMeasurement> &gps_measurements) {
  sym::Optimizerd optimizer(sym::DefaultOptimizerParams(), factors);

  // Build Values
  sym::Valuesd values;
  for (int i = 0; i < key_frame_times.size(); i++) {
    values.Set({Var::POSE, i},
               sym::Pose3d(sym::Rot3d(), gps_measurements[i].position));
    // values.Set({Var::POSE, i}, sym::Pose3d());

    values.Set({Var::VELOCITY, i}, Eigen::Vector3d::Zero());
  }
  for (int i = 0; i < key_frame_times.size() - 1; i++) {
    values.Set({Var::ACCEL_BIAS, i}, GetAccelBiasEstimate(key_frame_times[i]));
    values.Set({Var::GYRO_BIAS, i}, GetGyroBiasEstimate(key_frame_times[i]));
  }

  // gravity should point towards the direction of acceleration
  // ENU
  values.Set({Var::GRAVITY}, Eigen::Vector3d(0.0, 0.0, 0));
  values.Set({Var::EPSILON}, sym::kDefaultEpsilond);

  visualizeTrajectory(values, gps_measurements);

  optimizer.Optimize(values);
  return values;
}

int main() {

  std::vector<ImuMeasurement> imu_measurements;
  std::vector<GpsMeasurement> gps_measurements;
  createExampleStraightTrajectory(imu_measurements, gps_measurements);
  visualizeData(imu_measurements, gps_measurements, 10);

  std::vector<double> key_frame_times = GetKeyFrameTimes(gps_measurements);

  std::vector<sym::Factord> factors =
      createIMUfactors(key_frame_times, imu_measurements);

  sym::Valuesd optimized_values =
      optimizeImu(factors, key_frame_times, gps_measurements);

  visualizeTrajectory(optimized_values, gps_measurements);

  return 0;
}