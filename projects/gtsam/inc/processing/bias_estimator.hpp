/// @file processing/bias_estimator.hpp
/// @brief Initial IMU bias estimation from IMU vs odometry comparison.

#pragma once

#include <gtsam/navigation/ImuBias.h>
#include <vector>

#include "domain/measurements.hpp"

namespace nufuse::processing {

/// @brief Estimate initial gyro bias by comparing IMU readings with odometry rotation rates.
/// Accelerometer bias is left at zero (gravity-aligned, hard to estimate without motion model).
gtsam::imuBias::ConstantBias estimateInitialBias(const std::vector<domain::ImuMeasurement>& imu,
                                                 const std::vector<domain::OdomMeasurement>& odom);

}  // namespace nufuse::processing
