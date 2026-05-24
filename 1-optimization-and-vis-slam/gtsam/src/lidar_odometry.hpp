/// @file lidar_odometry.hpp
/// @brief LiDAR scan-to-scan GICP registration producing body-frame relative poses.

#pragma once

#include "nuscenes_domain_types.hpp"

#include "foxglove/PointCloud.pb.h"

#include <gtsam/geometry/Pose3.h>

#include <cstdint>
#include <vector>

/// @brief Result of a single scan-to-scan GICP alignment.
struct LidarRelativePose {
    uint64_t stamp_from;
    uint64_t stamp_to;
    slam::geometry::Transform<gtsam::Pose3, LidarTopFrame, LidarTopFrame> T_lidar;
    slam::geometry::Transform<gtsam::Pose3, BodyFrame, BodyFrame> T_body;
    bool converged;
    double error;
    int num_inliers;
};

/// @brief Compute scan-to-scan GICP relative poses for all consecutive scans.
///
/// @param lidar_scans     Timestamped point clouds sorted chronologically.
/// @param body_from_lidar Extrinsic calibration (body ← lidar_top).
/// @return Vector of relative poses (one per consecutive pair).
std::vector<LidarRelativePose> computeLidarOdometry(
    const std::vector<Stamped<foxglove::PointCloud>>& lidar_scans,
    const slam::geometry::Transform<gtsam::Pose3, BodyFrame, LidarTopFrame>& body_from_lidar);
