/// @file nuscenes_domain_types.hpp
/// @brief Frame tags and type-safe wrappers for NuScenes sensor measurements.
///
/// Design: protobuf messages are the raw data layer. This header defines the
/// domain boundary where raw data gets wrapped into frame-aware types suitable
/// for GTSAM factor construction. Measurements that are inherently
/// frame-identified at runtime (point clouds, images) stay as protobuf.
/// Static sensor extrinsics are typed via ExtrinsicCalibration.

#pragma once

#include <slam/core/strong_id.hpp>
#include <slam/geometry/safe_geometry.hpp>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

#include <cstdint>
#include <optional>
#include <vector>

// ============================================================================
// Coordinate frame tags
// ============================================================================

/// @brief Earth-Centered WGS-84 geodetic frame (lat/lon/alt).
struct Wgs84Frame {};

/// @brief Local East-North-Up tangent frame anchored at a reference point.
struct EnuFrame {};

/// @brief Map / world frame (NuScenes scene-global coordinates).
struct MapFrame {};

/// @brief Vehicle body frame (IMU-aligned, rear axle on ground).
struct BodyFrame {};

/// @brief IMU sensor frame (coincides with BodyFrame in NuScenes).
struct ImuFrame {};

// ---- Sensor frames (static extrinsics from body) --------------------------

/// @brief LIDAR_TOP sensor frame.
struct LidarTopFrame {};

/// @brief CAM_FRONT sensor frame.
struct CamFrontFrame {};
/// @brief CAM_FRONT_LEFT sensor frame.
struct CamFrontLeftFrame {};
/// @brief CAM_FRONT_RIGHT sensor frame.
struct CamFrontRightFrame {};
/// @brief CAM_BACK sensor frame.
struct CamBackFrame {};
/// @brief CAM_BACK_LEFT sensor frame.
struct CamBackLeftFrame {};
/// @brief CAM_BACK_RIGHT sensor frame.
struct CamBackRightFrame {};

/// @brief RADAR_FRONT sensor frame.
struct RadarFrontFrame {};
/// @brief RADAR_FRONT_LEFT sensor frame.
struct RadarFrontLeftFrame {};
/// @brief RADAR_FRONT_RIGHT sensor frame.
struct RadarFrontRightFrame {};
/// @brief RADAR_BACK_LEFT sensor frame.
struct RadarBackLeftFrame {};
/// @brief RADAR_BACK_RIGHT sensor frame.
struct RadarBackRightFrame {};

// ============================================================================
// Identifiers
// ============================================================================

struct TimestampTag {};
/// @brief Nanosecond-precision log timestamp used as a factor-graph key.
using TimestampId = slam::core::StrongId<TimestampTag, uint64_t>;

// ============================================================================
// IMU measurement (body-frame)
// ============================================================================

/// @brief A single IMU sample with body-frame accelerometer and gyroscope.
struct ImuMeasurement {
    TimestampId stamp;
    slam::geometry::TaggedPoint<gtsam::Vector3, ImuFrame> linear_acceleration;
    slam::geometry::TaggedPoint<gtsam::Vector3, ImuFrame> angular_velocity;
    gtsam::Rot3 orientation;  ///< Raw CAN bus orientation (not frame-tagged).
};

// ============================================================================
// Odometry (vehicle pose in map)
// ============================================================================

/// @brief Vehicle odometry from CAN bus: pose + velocity in map frame.
struct OdomMeasurement {
    TimestampId stamp;
    slam::geometry::Pose<gtsam::Pose3, BodyFrame, MapFrame> pose;
    slam::geometry::TaggedPoint<gtsam::Vector3, BodyFrame> velocity;
    slam::geometry::TaggedPoint<gtsam::Vector3, BodyFrame> acceleration;
    slam::geometry::TaggedPoint<gtsam::Vector3, BodyFrame> angular_velocity;
};

// ============================================================================
// GNSS fix (WGS-84 geodetic)
// ============================================================================

/// @brief GNSS navigation fix in WGS-84 geodetic coordinates.
struct GnssFix {
    TimestampId stamp;
    /// @brief (latitude°, longitude°, altitude_m) in WGS-84.
    slam::geometry::TaggedPoint<gtsam::Point3, Wgs84Frame> lla;
    std::vector<double> position_covariance;  ///< Row-major 3×3 (if available).
};

// ============================================================================
// Stamped protobuf wrapper (for data that stays as raw protobuf)
// ============================================================================

/// @brief Thin wrapper that pairs a message timestamp with a protobuf payload.
///
/// Used for point clouds and images — data types where the frame identity
/// is a runtime string, not a compile-time tag.
template <typename ProtoMsg>
struct Stamped {
    TimestampId stamp;
    ProtoMsg msg;
};

// ============================================================================
// Static sensor extrinsics (body → sensor)
// ============================================================================

/// @brief Static extrinsic calibration for all NuScenes sensors.
///
/// Extracted from `/tf` FrameTransform messages where `parent_frame_id` is
/// "base_link". These transforms are constant for a given vehicle setup.
/// The `map → base_link` dynamic pose is NOT included here (it's in odom).
///
/// Fields are `std::optional` so that missing sensors can be detected.
struct ExtrinsicCalibration {
    // LiDAR
    std::optional<slam::geometry::Transform<gtsam::Pose3, BodyFrame, LidarTopFrame>>
        body_from_lidar_top;

    // Cameras
    std::optional<slam::geometry::Transform<gtsam::Pose3, BodyFrame, CamFrontFrame>>
        body_from_cam_front;
    std::optional<slam::geometry::Transform<gtsam::Pose3, BodyFrame, CamFrontLeftFrame>>
        body_from_cam_front_left;
    std::optional<slam::geometry::Transform<gtsam::Pose3, BodyFrame, CamFrontRightFrame>>
        body_from_cam_front_right;
    std::optional<slam::geometry::Transform<gtsam::Pose3, BodyFrame, CamBackFrame>>
        body_from_cam_back;
    std::optional<slam::geometry::Transform<gtsam::Pose3, BodyFrame, CamBackLeftFrame>>
        body_from_cam_back_left;
    std::optional<slam::geometry::Transform<gtsam::Pose3, BodyFrame, CamBackRightFrame>>
        body_from_cam_back_right;

    // Radars
    std::optional<slam::geometry::Transform<gtsam::Pose3, BodyFrame, RadarFrontFrame>>
        body_from_radar_front;
    std::optional<slam::geometry::Transform<gtsam::Pose3, BodyFrame, RadarFrontLeftFrame>>
        body_from_radar_front_left;
    std::optional<slam::geometry::Transform<gtsam::Pose3, BodyFrame, RadarFrontRightFrame>>
        body_from_radar_front_right;
    std::optional<slam::geometry::Transform<gtsam::Pose3, BodyFrame, RadarBackLeftFrame>>
        body_from_radar_back_left;
    std::optional<slam::geometry::Transform<gtsam::Pose3, BodyFrame, RadarBackRightFrame>>
        body_from_radar_back_right;
};

