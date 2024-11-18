#include "loadKittiData.h"

bool openFile(std::ifstream &file, const std::string &filename) {
  file.open(filename);
  if (!file.is_open()) {
    spdlog::error("Failed to open file: {}", filename);
    return false;
  }
  return true;
}

void loadKittiData(KittiCalibration &kitti_calibration, std::vector<ImuMeasurement> &imu_measurements, std::vector<GpsMeasurement> &gps_measurements) {
  std::string line;
  std::string imu_data_file = "../data/KittiEquivBiasedImu.txt";
  std::string imu_metadata_file = "../data/KittiEquivBiasedImu_metadata.txt";
  std::string gps_data_file = "../data/KittiGps_converted.txt";

  std::ifstream imu_metadata, imu_data, gps_data;

  if (!openFile(imu_metadata, imu_metadata_file) || !openFile(imu_data, imu_data_file) || !openFile(gps_data, gps_data_file)) {
    return;
  }

  getline(imu_metadata, line);  // Skip the first line (header)

  getline(imu_metadata, line);  // Load Kitti calibration data
  int num_params = sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &kitti_calibration.body_ptx, &kitti_calibration.body_pty, &kitti_calibration.body_ptz,
                          &kitti_calibration.body_prx, &kitti_calibration.body_pry, &kitti_calibration.body_prz, &kitti_calibration.accelerometer_sigma, &kitti_calibration.gyroscope_sigma,
                          &kitti_calibration.integration_sigma, &kitti_calibration.accelerometer_bias_sigma, &kitti_calibration.gyroscope_bias_sigma, &kitti_calibration.average_delta_t);
  if (num_params != 12) {
    spdlog::error("Failed to parse IMU metadata line: {}", line);
    return;
  }

  spdlog::info("IMU metadata: {} {} {} {} {} {} {} {} {} {} {} {}", kitti_calibration.body_ptx, kitti_calibration.body_pty, kitti_calibration.body_ptz, kitti_calibration.body_prx,
               kitti_calibration.body_pry, kitti_calibration.body_prz, kitti_calibration.accelerometer_sigma, kitti_calibration.gyroscope_sigma, kitti_calibration.integration_sigma,
               kitti_calibration.accelerometer_bias_sigma, kitti_calibration.gyroscope_bias_sigma, kitti_calibration.average_delta_t);

  getline(imu_data, line);  // Skip the first line (header)
  double time, dt, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;
  while (getline(imu_data, line)) {
    int num_values = sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf", &time, &dt, &acc_x, &acc_y, &acc_z, &gyro_x, &gyro_y, &gyro_z);
    if (num_values != 8) {
      spdlog::warn("Failed to parse IMU data line: {}", line);
      continue;
    }
    imu_measurements.push_back({time, dt, Eigen::Vector3d(acc_x, acc_y, acc_z), Eigen::Vector3d(gyro_x, gyro_y, gyro_z)});
  }

  getline(gps_data, line);  // Skip the first line (header)
  double gps_time, gps_x, gps_y, gps_z;
  while (getline(gps_data, line)) {
    int num_values = sscanf(line.c_str(), "%lf,%lf,%lf,%lf", &gps_time, &gps_x, &gps_y, &gps_z);
    if (num_values != 4) {
      spdlog::warn("Failed to parse GPS data line: {}", line);
      continue;
    }
    gps_measurements.push_back({gps_time, Eigen::Vector3d(gps_x, gps_y, gps_z)});
  }
}
