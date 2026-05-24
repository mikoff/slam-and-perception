/// @file processing/geo_utils.hpp
/// @brief Geodetic and odometry interpolation utilities.

#pragma once

#include "domain/measurements.hpp"

#include <vector>

namespace nufuse::processing {

/// @brief Convert WGS-84 geodetic coordinates to local ENU.
gtsam::Point3 llaToEnu(double lat, double lon, double alt,
                        double lat0, double lon0, double alt0);

/// @brief Interpolate odometry at the given timestamp using SE(3) geodesic.
domain::OdomMeasurement interpolateOdom(
    const std::vector<domain::OdomMeasurement>& odom, uint64_t stamp_ns);

}  // namespace nufuse::processing
