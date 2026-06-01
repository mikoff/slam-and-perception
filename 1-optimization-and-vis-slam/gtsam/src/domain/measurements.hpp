/// @file domain/measurements.hpp
/// @brief Type-safe sensor measurement structures.

#pragma once

#include "core/types.hpp"

#include <gtsam/geometry/Rot3.h>

#include <vector>

namespace nufuse::domain {

/// @brief Single IMU sample with body-frame accelerometer and gyroscope.
struct ImuMeasurement {
    core::Timestamp stamp;
    core::ImuVector linear_acceleration;
    core::ImuVector angular_velocity;
    core::BodyOrientationEnu orientation;
};

/// @brief Vehicle odometry: pose + dynamics in body/map frames.
struct OdomMeasurement {
    core::Timestamp stamp;
    core::BodyInMap pose;
    core::BodyVector velocity;
    core::BodyVector acceleration;
    core::BodyVector angular_velocity;
};

/// @brief GNSS fix in WGS-84 geodetic coordinates.
struct GnssFix {
    core::Timestamp stamp;
    core::Wgs84Lla lla;
    std::vector<double> position_covariance;
};

}  // namespace nufuse::domain
