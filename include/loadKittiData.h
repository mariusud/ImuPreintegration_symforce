#ifndef LOAD_KITTI_DATA_H
#define LOAD_KITTI_DATA_H

#include <Eigen/Dense>
#include <vector>

struct KittiCalibration {
  double body_ptx;
  double body_pty;
  double body_ptz;
  double body_prx;
  double body_pry;
  double body_prz;
  double accelerometer_sigma;
  double gyroscope_sigma;
  double integration_sigma;
  double accelerometer_bias_sigma;
  double gyroscope_bias_sigma;
  double average_delta_t;
};

struct ImuMeasurement {
  double time;
  double dt;
  Eigen::Vector3d accelerometer;
  Eigen::Vector3d gyroscope; // omega
};

struct GpsMeasurement {
  double time;
  Eigen::Vector3d position; // x,y,z
};

void loadKittiData(KittiCalibration &kitti_calibration,
                   std::vector<ImuMeasurement> &imu_measurements,
                   std::vector<GpsMeasurement> &gps_measurements);

std::string findExampleDataFile(const std::string &name);

#endif // LOAD_KITTI_DATA_H
