#include "createFactors.h"

/**
 * @brief Creates a factor to model the Accelerometer bias between two consecutive time steps.
 *
 * The keys optimized for this factor are `ACCEL_BIAS.WithSuper(i)` and `ACCEL_BIAS.WithSuper(i + 1)`.
 *
 * Residual = (time_delta * diagonal_information) * (prev_bias - next_bias)
 *
 * @param i The current factor index.
 * @return sym::Factord factor
 */
sym::Factord createAccelBiasFactor(const int i) {
  return sym::Factord::Hessian(sym::BiasBetweenFactor<double>,
                               {sym::Keys::ACCEL_BIAS_DIAG_SQRT_INFO, sym::Keys::TIME_DELTA.WithSuper(i), sym::Keys::ACCEL_BIAS.WithSuper(i - 1), sym::Keys::ACCEL_BIAS.WithSuper(i)},
                               {sym::Keys::ACCEL_BIAS.WithSuper(i - 1), sym::Keys::ACCEL_BIAS.WithSuper(i)});
}

/**
 * @brief Creates a factor to model the Gyroscope bias between two consecutive time steps.
 *
 * The keys to optimize for this factor are `GYRO_BIAS.WithSuper(i)` and `GYRO_BIAS.WithSuper(i+1)`
 *
 * Residual = (time_delta * diagonal_information) * (prev_bias - next_bias)
 *
 * @param i The current factor index.
 * @return sym::Factord
 */
sym::Factord createGyroBiasFactor(const int i) {
  return sym::Factord::Hessian(sym::BiasBetweenFactor<double>,
                               {sym::Keys::GYRO_BIAS_DIAG_SQRT_INFO, sym::Keys::TIME_DELTA.WithSuper(i), sym::Keys::GYRO_BIAS.WithSuper(i - 1), sym::Keys::GYRO_BIAS.WithSuper(i)},
                               {sym::Keys::GYRO_BIAS.WithSuper(i - 1), sym::Keys::GYRO_BIAS.WithSuper(i)});
}

// TODO could use PriorFactorPose3Position instead
sym::Factord createPoseFactor(const int i) {
  return sym::Factord::Hessian(sym::PriorFactorPose3<double>, {sym::Keys::POSE.WithSuper(i), sym::Keys::MEASURED_POSE.WithSuper(i), sym::Keys::SQRT_INFO, sym::Keys::EPSILON},
                               {sym::Keys::POSE.WithSuper(i)});
}
sym::Factord createImuFactor(const int i, sym::ImuPreintegrator<double> integrator) {
  return sym::ImuFactor<double>(integrator)
      .Factor({sym::Keys::POSE.WithSuper(i - 1), sym::Keys::VELOCITY.WithSuper(i - 1), sym::Keys::POSE.WithSuper(i), sym::Keys::VELOCITY.WithSuper(i), sym::Keys::ACCEL_BIAS.WithSuper(i),
               sym::Keys::GYRO_BIAS.WithSuper(i), sym::Keys::GRAVITY, sym::Keys::EPSILON});
}
