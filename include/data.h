#ifndef DATA_H
#define DATA_H
#include <Eigen/Dense>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <random>
#include <vector>

#include <sym/pose3.h>
#include <symforce/opt/factor.h>

enum Var : char {
  POSE = 'p',       // Pose3d
  VELOCITY = 'v',   // Vector3d
  ACCEL_BIAS = 'A', // Vector3d
  GYRO_BIAS = 'G',  // Vector3d
  GRAVITY = 'g',    // Vector3d
  EPSILON = 'e'     // Scalar
};

struct ImuMeasurement {
  Eigen::Vector3d acceleration;
  Eigen::Vector3d angular_velocity;
  double timestamp;
};

struct GpsMeasurement {
  double time;
  Eigen::Vector3d position; // x, y, z
};

void createExampleStraightTrajectory(
    std::vector<ImuMeasurement> &imu_measurements,
    std::vector<GpsMeasurement> &gps_measurements);

void visualizeData(const std::vector<ImuMeasurement> &imu_measurements,
                   const std::vector<GpsMeasurement> &gps_measurements,
                   size_t num_to_visualize);

void visualizeTrajectory(const sym::Valuesd &optimized_values,
                         const std::vector<GpsMeasurement> &gps_measurements);

#endif // DATA_H
