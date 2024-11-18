#ifndef LOAD_KITTI_DATA_H
#define LOAD_KITTI_DATA_H

#include <spdlog/spdlog.h>

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <string>
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
  Eigen::Vector3d gyroscope;
};

struct GpsMeasurement {
  double time;
  Eigen::Vector3d position;
};

bool openFile(std::ifstream &file, const std::string &filename);

void loadKittiData(KittiCalibration &kitti_calibration, std::vector<ImuMeasurement> &imu_measurements, std::vector<GpsMeasurement> &gps_measurements);

#endif  // LOAD_KITTI_DATA_H
