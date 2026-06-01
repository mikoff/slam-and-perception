/// @file graph/factor_types.hpp
/// @brief Type-safe factor types using Timestamp as the primary key.

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>

#include "core/gtsam_boundary.hpp"
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

/// @brief A single radar radial velocity measurement for the spatiotemporal factor.
struct StoredRadarMeasurement {
  double v_radial;                    // measured radial velocity [m/s]
  core::RadarBearing bearing;         // unit bearing vector in radar frame (z-squashed)
  core::AngularVelocityBody omega_B;  // angular velocity in body frame at measurement time
};

/// @brief Radar factor: all inlier measurements at a single keyframe timestamp.
struct StoredRadarFactor {
  core::Timestamp stamp;
  gtsam::Key extrinsic_key;  // symbol for the radar extrinsic being estimated
  std::vector<StoredRadarMeasurement> measurements;
};

/// @brief Non-holonomic constraint at a keyframe.
struct StoredNhcFactor {
  core::Timestamp stamp;
  core::AngularVelocityBody omega_B;  // angular velocity in body frame at this keyframe
};

/// @brief Forward velocity measurement between consecutive keyframes.
struct StoredForwardVelocityFactor {
  core::Timestamp stamp_from;
  core::Timestamp stamp_to;
  double measured_v;                  // forward speed from odometry [m/s]
  double dt;                          // time between keyframes [s]
  core::AngularVelocityBody omega_B;  // angular velocity at stamp_from
};

}  // namespace nufuse::graph
