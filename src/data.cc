
#include "data.h"

// Simulate a straight-line trajectory with noise
void createExampleStraightTrajectory(
    std::vector<ImuMeasurement> &imu_measurements,
    std::vector<GpsMeasurement> &gps_measurements) {

  const double imu_frequency = 100.0;            // IMU updates at 100 Hz
  const double gps_frequency = 10.0;             // GPS updates at 10 Hz
  const double duration = 10.0;                  // Total time in seconds
  const Eigen::Vector3d velocity(1.0, 0.0, 0.0); // Robot moving in +X direction

  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<> noise(0.0, 0.01);

  for (double t = 0.0; t <= duration; t += 1.0 / imu_frequency) {
    ImuMeasurement imu;
    imu.timestamp = t;

    // Assume perfect acceleration along X, with small noise
    imu.acceleration = Eigen::Vector3d(0.0, 0.0, 0.0);
    imu.angular_velocity =
        Eigen::Vector3d(0.0, 0.0, 0.0) + Eigen::Vector3d::Random() * 0.01;
    imu_measurements.push_back(imu);
  }

  for (double t = 0.0; t <= duration; t += 1.0 / gps_frequency) {
    GpsMeasurement gps;
    gps.time = t;
    gps.position =
        velocity * t + Eigen::Vector3d(noise(gen), noise(gen), noise(gen));
    gps_measurements.push_back(gps);
  }
}

void visualizeTrajectory(const sym::Valuesd &optimized_values,
                         const std::vector<GpsMeasurement> &gps_measurements) {

  pcl::visualization::PCLVisualizer viewer("Trajectory Viewer");

  // Create point clouds for IMU and GPS
  pcl::PointCloud<pcl::PointXYZ>::Ptr optimized_imu_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr gps_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());

  // Add optimized IMU points (trajectory) from optimized values
  for (size_t i = 0; i < gps_measurements.size(); ++i) {
    sym::Pose3d pose =
        optimized_values.At<sym::Pose3d>({Var::POSE, static_cast<int>(i)});
    pcl::PointXYZ point;
    point.x = pose.Position()[0];
    point.y = pose.Position()[1];
    point.z = pose.Position()[2];
    optimized_imu_cloud->points.push_back(point);

    const auto gps = gps_measurements[i];
    pcl::PointXYZ gps_point;
    gps_point.x = gps.position[0];
    gps_point.y = gps.position[1];
    gps_point.z = gps.position[2];
    gps_cloud->points.push_back(gps_point);
    std::cout << "IMU Pose: " << pose.Position().transpose() << "\t"
              << "GPS Pose: " << gps.position.transpose() << std::endl;
  }

  viewer.addPointCloud(optimized_imu_cloud, "optimized imu cloud");
  viewer.addPointCloud(gps_cloud, "gps cloud");

  // Create line segments for the Optimized IMU trajectory
  for (size_t i = 1; i < optimized_imu_cloud->points.size(); ++i) {
    pcl::PointXYZ start_point = optimized_imu_cloud->points[i - 1];
    pcl::PointXYZ end_point = optimized_imu_cloud->points[i];
    viewer.addLine(start_point, end_point, 1, 1, 1,
                   "imu line" + std::to_string(i));
  }

  for (size_t i = 1; i < gps_measurements.size(); ++i) {
    pcl::PointXYZ start_point, end_point;
    start_point.x = gps_measurements[i - 1].position[0];
    start_point.y = gps_measurements[i - 1].position[1];
    start_point.z = gps_measurements[i - 1].position[2];
    end_point.x = gps_measurements[i].position[0];
    end_point.y = gps_measurements[i].position[1];
    end_point.z = gps_measurements[i].position[2];
    viewer.addLine(start_point, end_point, 0, 1, 0,
                   "gps line" + std::to_string(i));
  }

  // Start the visualizer
  while (!viewer.wasStopped()) {
    viewer.spinOnce(0);
  }
}

void visualizeData(const std::vector<ImuMeasurement> &imu_measurements,
                   const std::vector<GpsMeasurement> &gps_measurements,
                   size_t num_to_visualize) {

  // Limit the number of measurements to display
  size_t gps_to_display = std::min(num_to_visualize, gps_measurements.size());
  size_t imu_to_display = std::min(num_to_visualize, imu_measurements.size());

  std::cout << "GPS Measurements:\n";
  std::cout << std::setw(10) << "Time" << std::setw(15) << "Position (X)"
            << std::setw(15) << "Position (Y)" << std::setw(15)
            << "Position (Z)"
            << "\n";

  for (size_t i = 0; i < gps_to_display; ++i) {
    const auto &gps = gps_measurements[i];
    std::cout << std::fixed << std::setprecision(4) << std::setw(10) << gps.time
              << std::setw(15) << gps.position.x() << std::setw(15)
              << gps.position.y() << std::setw(15) << gps.position.z() << "\n";
  }

  std::cout << "\nIMU Measurements:\n";
  std::cout << std::setw(10) << "Time" << std::setw(15) << "Accel (X)"
            << std::setw(15) << "Accel (Y)" << std::setw(15) << "Accel (Z)"
            << std::setw(15) << "AngVel (X)" << std::setw(15) << "AngVel (Y)"
            << std::setw(15) << "AngVel (Z)"
            << "\n";

  for (size_t i = 0; i < imu_to_display; ++i) {
    const auto &imu = imu_measurements[i];
    std::cout << std::fixed << std::setprecision(4) << std::setw(10)
              << imu.timestamp << std::setw(15) << imu.acceleration.x()
              << std::setw(15) << imu.acceleration.y() << std::setw(15)
              << imu.acceleration.z() << std::setw(15)
              << imu.angular_velocity.x() << std::setw(15)
              << imu.angular_velocity.y() << std::setw(15)
              << imu.angular_velocity.z() << "\n";
  }
}