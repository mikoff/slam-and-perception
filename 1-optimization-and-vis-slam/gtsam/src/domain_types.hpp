/// @file domain_types.hpp
/// @brief Frame tags and type aliases for the 2-D pose-graph example.

#pragma once

#include <slam/core/strong_id.hpp>
#include <slam/geometry/safe_geometry.hpp>

#include <gtsam/geometry/Pose2.h>

// ============================================================================
// Coordinate frame tags
// ============================================================================

struct WorldFrame {};
struct VehicleFrame {};

// ============================================================================
// Identifiers
// ============================================================================

struct VehicleTag {};
/// @brief Strongly-typed node key. Prevents mixing pose IDs with landmark IDs.
using VehicleId = slam::core::StrongId<VehicleTag>;

// ============================================================================
// Geometry aliases
// ============================================================================

/// @brief T_{World←Vehicle}: the vehicle's pose expressed in the world frame.
using VehiclePoseInWorld = slam::geometry::Pose<gtsam::Pose2, VehicleFrame, WorldFrame>;

/// @brief Relative transform between two vehicle poses (odometry / loop-closure).
using RelativeVehicleTransform = slam::geometry::Transform<gtsam::Pose2, VehicleFrame, VehicleFrame>;
