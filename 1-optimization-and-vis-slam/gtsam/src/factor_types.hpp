/// @file factor_types.hpp
/// @brief Type-safe factor and keyframe types for the NuScenes factor graph.
///
/// Defines the domain boundary between raw sensor measurements and the
/// abstract factor graph.  All stored factors use frame-tagged types and
/// strongly-typed KeyframeIds — no raw integers leak across module boundaries.

#pragma once

#include "nuscenes_domain_types.hpp"

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>

#include <optional>

// ============================================================================
// Keyframe identifier
// ============================================================================

struct KeyframeTag {};

/// @brief Strongly-typed keyframe index (maps 1:1 to GTSAM symbol index).
using KeyframeId = slam::core::StrongId<KeyframeTag, int>;

// ============================================================================
// Initial estimate for a keyframe
// ============================================================================

/// @brief Linearization-point values for one keyframe in the factor graph.
struct KeyframeEstimate {
    KeyframeId id;
    slam::geometry::Pose<gtsam::Pose3, BodyFrame, EnuFrame> pose;
    slam::geometry::TaggedPoint<gtsam::Vector3, EnuFrame> velocity;
    gtsam::imuBias::ConstantBias bias;
};

// ============================================================================
// Stored factor types
// ============================================================================

/// @brief Prior constraints on the first keyframe (pose, velocity, bias, GPS).
struct PriorFactor {
    KeyframeId keyframe;
    slam::geometry::Pose<gtsam::Pose3, BodyFrame, EnuFrame> pose;
    slam::geometry::TaggedPoint<gtsam::Vector3, EnuFrame> velocity;
    gtsam::imuBias::ConstantBias bias;
    slam::geometry::TaggedPoint<gtsam::Point3, EnuFrame> gps_position;
};

/// @brief IMU preintegration factor between two consecutive keyframes.
struct StoredImuFactor {
    KeyframeId from;
    KeyframeId to;
    gtsam::PreintegratedCombinedMeasurements pim;
};

/// @brief GNSS position factor at a specific keyframe (ENU coordinates).
struct StoredGpsFactor {
    KeyframeId keyframe;
    slam::geometry::TaggedPoint<gtsam::Point3, EnuFrame> position_enu;
    bool corrupted = false;
};

/// @brief LiDAR-derived relative pose factor between two keyframes (body frame).
struct StoredLidarFactor {
    KeyframeId from;
    KeyframeId to;
    slam::geometry::Transform<gtsam::Pose3, LidarTopFrame, LidarTopFrame> relative_pose;
};
