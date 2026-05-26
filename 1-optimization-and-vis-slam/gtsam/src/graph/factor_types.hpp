/// @file graph/factor_types.hpp
/// @brief Type-safe factor types using Timestamp as the primary key.

#pragma once

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>

#include "core/types.hpp"

namespace nufuse::graph {

/// @brief Linearization-point values for one keyframe.
struct KeyframeEstimate {
  core::Timestamp stamp;
  core::BodyInEnu pose;
  core::EnuVelocity velocity;
  gtsam::imuBias::ConstantBias bias;
};

/// @brief Prior constraints on the first keyframe.
struct PriorFactor {
  core::Timestamp stamp;
  core::BodyInEnu pose;
  core::EnuVelocity velocity;
  gtsam::imuBias::ConstantBias bias;
  core::EnuPosition gps_position;
};

/// @brief IMU preintegration factor between two consecutive keyframes.
struct StoredImuFactor {
  core::Timestamp stamp_from;
  core::Timestamp stamp_to;
  gtsam::PreintegratedCombinedMeasurements pim;
};

/// @brief GNSS position factor at a specific keyframe (ENU coordinates).
struct StoredGpsFactor {
  core::Timestamp stamp;
  core::EnuPosition position_enu;
  bool corrupted = false;
};

/// @brief LiDAR-derived relative pose factor between two keyframes.
struct StoredLidarFactor {
  core::Timestamp stamp_from;
  core::Timestamp stamp_to;
  core::LidarDelta relative_pose;
};

}  // namespace nufuse::graph
