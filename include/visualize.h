#ifndef DATA_H
#define DATA_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sym/pose3.h>
#include <symforce/opt/factor.h>

#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <vector>

#include "../gen/keys.h"
#include "loadKittiData.h"

void createExampleStraightTrajectory(std::vector<ImuMeasurement> &imu_measurements, std::vector<GpsMeasurement> &gps_measurements);

void visualizeData(const std::vector<ImuMeasurement> &imu_measurements, const std::vector<GpsMeasurement> &gps_measurements, size_t num_to_visualize);

void visualizeTrajectory(const sym::Valuesd &optimized_values, const std::vector<GpsMeasurement> &gps_measurements, int NUM_FACTORS);

#endif  // DATA_H
