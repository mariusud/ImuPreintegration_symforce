// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     keys.h.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <symforce/opt/key.h>

namespace sym {

namespace Keys {

static const sym::Key PRIOR_POSE = 'p';                  // sym::Pose3<Scalar>
static const sym::Key PRIOR_VELOCITY = 'P';              // Eigen::Matrix<Scalar, 3, 1>
static const sym::Key PRIOR_ACCEL_BIAS = 'a';            // Eigen::Matrix<Scalar, 3, 1>
static const sym::Key PRIOR_GYRO_BIAS = 'g';             // Eigen::Matrix<Scalar, 3, 1>
static const sym::Key POSE = 'o';                        // sym::Pose3<Scalar>
static const sym::Key MEASURED_POSE = 'm';               // sym::Pose3<Scalar>
static const sym::Key TIME_DELTA = 't';                  // Scalar
static const sym::Key VELOCITY = 'v';                    // Eigen::Matrix<Scalar, 3, 1>
static const sym::Key VELOCITY_PRIOR_SQRT_INFO = 'V';    // Eigen::Matrix<Scalar, 3, 3>
static const sym::Key ACCEL_COV = 'A';                   // Eigen::Matrix<Scalar, 3, 1>
static const sym::Key GYRO_COV = 'G';                    // Eigen::Matrix<Scalar, 3, 1>
static const sym::Key ACCEL_BIAS = 'b';                  // Eigen::Matrix<Scalar, 3, 1>
static const sym::Key GYRO_BIAS = 'B';                   // Eigen::Matrix<Scalar, 3, 1>
static const sym::Key GRAVITY = 'r';                     // Eigen::Matrix<Scalar, 3, 1>
static const sym::Key ACCEL_BIAS_PRIOR_SQRT_INFO = 's';  // Eigen::Matrix<Scalar, 3, 3>
static const sym::Key GYRO_BIAS_PRIOR_SQRT_INFO = 'S';   // Eigen::Matrix<Scalar, 3, 3>
static const sym::Key ACCEL_BIAS_DIAG_SQRT_INFO = 'd';   // Eigen::Matrix<Scalar, 3, 1>
static const sym::Key GYRO_BIAS_DIAG_SQRT_INFO = 'D';    // Eigen::Matrix<Scalar, 3, 1>
static const sym::Key MEAS_POSE_SQRT_INFO = 'M';         // Eigen::Matrix<Scalar, 6, 6>
static const sym::Key POSE_PRIOR_SQRT_INFO = 'i';        // Eigen::Matrix<Scalar, 6, 6>
static const sym::Key EPSILON = 'e';                     // Scalar

}  // namespace Keys

}  // namespace sym
