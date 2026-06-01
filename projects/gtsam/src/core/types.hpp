/// @file core/types.hpp
/// @brief Central type aliases for transforms, poses, points, and timestamps.

#pragma once

#include "frames.hpp"

#include <slam/core/strong_id.hpp>
#include <slam/geometry/safe_geometry.hpp>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

#include <cstdint>

namespace nufuse::core {

// ============================================================================
// Timestamp
// ============================================================================

struct TimestampTag {};

/// @brief Nanosecond-precision timestamp used as the primary measurement key.
using Timestamp = slam::core::StrongId<TimestampTag, uint64_t>;

// ============================================================================
// Transform aliases (T_{Target <- Source})
// ============================================================================

/// @brief Body <- LidarTop extrinsic calibration.
using BodyFromLidar = slam::geometry::Transform<gtsam::Pose3, cf::Body, cf::LidarTop>;

/// @brief LidarTop <- Body (inverse extrinsic).
using LidarFromBody = slam::geometry::Transform<gtsam::Pose3, cf::LidarTop, cf::Body>;

/// @brief Relative motion in lidar frame (scan-to-scan).
using LidarDelta = slam::geometry::Transform<gtsam::Pose3, cf::LidarTop, cf::LidarTop>;

/// @brief Relative motion in body frame.
using BodyDelta = slam::geometry::Transform<gtsam::Pose3, cf::Body, cf::Body>;

/// @brief Map <- Body ego transform.
using MapFromBody = slam::geometry::Transform<gtsam::Pose3, cf::Map, cf::Body>;

// ============================================================================
// Pose aliases (entity in reference frame)
// ============================================================================

/// @brief Body pose in ENU frame.
using BodyInEnu = slam::geometry::Pose<gtsam::Pose3, cf::Body, cf::Enu>;

/// @brief Body pose in Map frame.
using BodyInMap = slam::geometry::Pose<gtsam::Pose3, cf::Body, cf::Map>;

// ============================================================================
// Rotation aliases
// ============================================================================

/// @brief Body orientation in ENU frame (rotation from Body to Enu).
using BodyOrientationEnu = slam::geometry::Transform<gtsam::Rot3, cf::Enu, cf::Body>;

// ============================================================================
// Tagged point aliases
// ============================================================================

using ImuVector    = slam::geometry::TaggedPoint<gtsam::Vector3, cf::Imu>;
using BodyVector   = slam::geometry::TaggedPoint<gtsam::Vector3, cf::Body>;
using EnuVelocity  = slam::geometry::TaggedPoint<gtsam::Vector3, cf::Enu>;
using EnuPosition  = slam::geometry::TaggedPoint<gtsam::Point3, cf::Enu>;
using Wgs84Lla     = slam::geometry::TaggedPoint<gtsam::Point3, cf::Wgs84>;

// ============================================================================
// Stamped wrapper
// ============================================================================

/// @brief Pairs a timestamp with a payload.
template <typename T>
struct Stamped {
    Timestamp stamp;
    T data;
};

// ============================================================================
// Camera extrinsic aliases
// ============================================================================

using BodyFromCamFront      = slam::geometry::Transform<gtsam::Pose3, cf::Body, cf::CamFront>;
using BodyFromCamFrontLeft  = slam::geometry::Transform<gtsam::Pose3, cf::Body, cf::CamFrontLeft>;
using BodyFromCamFrontRight = slam::geometry::Transform<gtsam::Pose3, cf::Body, cf::CamFrontRight>;
using BodyFromCamBack       = slam::geometry::Transform<gtsam::Pose3, cf::Body, cf::CamBack>;
using BodyFromCamBackLeft   = slam::geometry::Transform<gtsam::Pose3, cf::Body, cf::CamBackLeft>;
using BodyFromCamBackRight  = slam::geometry::Transform<gtsam::Pose3, cf::Body, cf::CamBackRight>;

// Radar extrinsic aliases
using BodyFromRadarFront      = slam::geometry::Transform<gtsam::Pose3, cf::Body, cf::RadarFront>;
using BodyFromRadarFrontLeft  = slam::geometry::Transform<gtsam::Pose3, cf::Body, cf::RadarFrontLeft>;
using BodyFromRadarFrontRight = slam::geometry::Transform<gtsam::Pose3, cf::Body, cf::RadarFrontRight>;
using BodyFromRadarBackLeft   = slam::geometry::Transform<gtsam::Pose3, cf::Body, cf::RadarBackLeft>;
using BodyFromRadarBackRight  = slam::geometry::Transform<gtsam::Pose3, cf::Body, cf::RadarBackRight>;

}  // namespace nufuse::core
