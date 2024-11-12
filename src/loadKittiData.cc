#include <cstring>
#include <fstream>
#include <iostream>

#include "loadKittiData.h"

using namespace std;

void loadKittiData(KittiCalibration &kitti_calibration,
                   std::vector<ImuMeasurement> &imu_measurements,
                   std::vector<GpsMeasurement> &gps_measurements) {
  string line;

  // Read IMU metadata and compute relative sensor pose transforms
  // BodyPtx BodyPty BodyPtz BodyPrx BodyPry BodyPrz AccelerometerSigma
  // GyroscopeSigma IntegrationSigma AccelerometerBiasSigma GyroscopeBiasSigma
  // AverageDeltaT
  string imu_metadata_file = ("../src/KittiEquivBiasedImu_metadata.txt");
  ifstream imu_metadata(imu_metadata_file.c_str());

  printf("-- Reading sensor metadata\n");

  getline(imu_metadata, line, '\n'); // ignore the first line

  // Load Kitti calibration
  getline(imu_metadata, line, '\n');
  sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
         &kitti_calibration.body_ptx, &kitti_calibration.body_pty,
         &kitti_calibration.body_ptz, &kitti_calibration.body_prx,
         &kitti_calibration.body_pry, &kitti_calibration.body_prz,
         &kitti_calibration.accelerometer_sigma,
         &kitti_calibration.gyroscope_sigma,
         &kitti_calibration.integration_sigma,
         &kitti_calibration.accelerometer_bias_sigma,
         &kitti_calibration.gyroscope_bias_sigma,
         &kitti_calibration.average_delta_t);
  printf("IMU metadata: %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
         kitti_calibration.body_ptx, kitti_calibration.body_pty,
         kitti_calibration.body_ptz, kitti_calibration.body_prx,
         kitti_calibration.body_pry, kitti_calibration.body_prz,
         kitti_calibration.accelerometer_sigma,
         kitti_calibration.gyroscope_sigma, kitti_calibration.integration_sigma,
         kitti_calibration.accelerometer_bias_sigma,
         kitti_calibration.gyroscope_bias_sigma,
         kitti_calibration.average_delta_t);

  // Read IMU data
  // Time dt accelX accelY accelZ omegaX omegaY omegaZ
  string imu_data_file = ("../src/KittiEquivBiasedImu.txt");
  printf("-- Reading IMU measurements from file\n");
  {
    ifstream imu_data(imu_data_file.c_str());
    getline(imu_data, line, '\n'); // ignore the first line

    double time = 0, dt = 0, acc_x = 0, acc_y = 0, acc_z = 0, gyro_x = 0,
           gyro_y = 0, gyro_z = 0;
    while (!imu_data.eof()) {
      getline(imu_data, line, '\n');
      sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf", &time, &dt,
             &acc_x, &acc_y, &acc_z, &gyro_x, &gyro_y, &gyro_z);

      ImuMeasurement measurement;
      measurement.time = time;
      measurement.dt = dt;
      measurement.accelerometer = Eigen::Vector3d(acc_x, acc_y, acc_z);
      measurement.gyroscope = Eigen::Vector3d(gyro_x, gyro_y, gyro_z);
      imu_measurements.push_back(measurement);
    }
  }

  // Read GPS data
  // Time,X,Y,Z
  string gps_data_file = ("../src/KittiGps_converted.txt");
  printf("-- Reading GPS measurements from file\n");
  {
    ifstream gps_data(gps_data_file.c_str());
    getline(gps_data, line, '\n'); // ignore the first line

    double time = 0, gps_x = 0, gps_y = 0, gps_z = 0;
    while (!gps_data.eof()) {
      getline(gps_data, line, '\n');
      sscanf(line.c_str(), "%lf,%lf,%lf,%lf", &time, &gps_x, &gps_y, &gps_z);

      GpsMeasurement measurement;
      measurement.time = time;
      measurement.position = Eigen::Vector3d(gps_x, gps_y, gps_z);
      gps_measurements.push_back(measurement);
    }
  }
}