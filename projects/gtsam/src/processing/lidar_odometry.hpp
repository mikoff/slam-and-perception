/// @file processing/lidar_odometry.hpp
/// @brief LiDAR scan-to-scan GICP registration producing body-frame relative poses.

#pragma once

#include "core/types.hpp"

#include "foxglove/PointCloud.pb.h"

#include <vector>

namespace nufuse::processing {

/// @brief Result of a single scan-to-scan GICP alignment.
struct LidarRelativePose {
    core::Timestamp stamp_from;
    core::Timestamp stamp_to;
    core::LidarDelta T_lidar;
    core::BodyDelta T_body;
    bool converged;
    double error;
    int num_inliers;
};

/// @brief Compute scan-to-scan GICP relative poses for all consecutive scans.
std::vector<LidarRelativePose> computeLidarOdometry(
    const std::vector<core::Stamped<foxglove::PointCloud>>& lidar_scans,
    const core::BodyFromLidar& body_from_lidar);

}  // namespace nufuse::processing
