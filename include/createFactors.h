#ifndef CREATE_FACTORS_H
#define CREATE_FACTORS_H
#include <sym/factors/between_factor_matrix31.h>
#include <sym/factors/prior_factor_matrix31.h>
#include <sym/factors/prior_factor_pose3.h>
#include <sym/factors/prior_factor_pose3_position.h>
#include <sym/pose3.h>
#include <sym/util/epsilon.h>
#include <symforce/opt/factor.h>
#include <symforce/opt/optimizer.h>
#include <symforce/slam/imu_preintegration/imu_factor.h>
#include <symforce/slam/imu_preintegration/imu_preintegrator.h>
#include <symforce/slam/imu_preintegration/preintegrated_imu_measurements.h>

#include "../gen/bias_between_factor.h"
#include "../gen/keys.h"

sym::Factord createAccelBiasFactor(const int i);
sym::Factord createGyroBiasFactor(const int i);
sym::Factord createImuFactor(const int i, sym::ImuPreintegrator<double> integrator);
sym::Factord createMeasuredPoseFactor(const int i);

// Prior factors
sym::Factord createAccelBiasPriorFactor(const int i);
sym::Factord createGyroBiasPriorFactor(const int i);
sym::Factord createPriorPoseFactor(const int i);
sym::Factord createVelocityPriorFactor(const int i);

#endif  // CREATE_FACTORS_H