/// @file results/optimizer.hpp
/// @brief Optimization and type-safe results unpacking.

#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <map>
#include <vector>

#include "core/gtsam_boundary.hpp"
#include "core/types.hpp"
#include "graph/builder.hpp"

namespace nufuse::results {

/// @brief A single optimized pose with associated covariance.
struct OptimizedPose {
  core::Timestamp stamp;
  core::BodyInEnu pose;
  Eigen::Matrix<double, 6, 6> covariance;
};

/// @brief A single optimized velocity with associated covariance.
struct OptimizedVelocity {
  core::Timestamp stamp;
  core::EnuVelocity velocity;
  Eigen::Matrix3d covariance;
};

/// @brief Optimized IMU bias.
struct OptimizedBias {
  core::Timestamp stamp;
  gtsam::imuBias::ConstantBias bias;
  Eigen::Matrix<double, 6, 6> covariance;
};

/// @brief Complete type-safe optimization results.
struct OptimizedResults {
  std::vector<OptimizedPose> poses;
  std::vector<OptimizedVelocity> velocities;
  std::vector<OptimizedBias> biases;
  core::BodyFromLidar body_from_lidar;
  Eigen::Matrix<double, 6, 6> lidar_extrinsics_covariance;
  /// @brief Estimated radar extrinsics keyed by RadarSensorId.
  std::map<core::RadarSensorId, gtsam::Pose3> radar_extrinsics;
  /// @brief Estimated odometry scale correction (scale = 1 + delta).
  core::OdomScaleDelta odom_scale_delta;
  int num_keyframes = 0;
  int num_corrupted_gnss = 0;
};

/// @brief Run optimization and unpack results.
/// Uses LM or GNC-LM depending on config.optimizer setting.
OptimizedResults optimize(const graph::FactorGraphBundle& gtsam_graph,
                          const graph::FactorStorage& storage,
                          const core::PipelineConfig& config = {});

}  // namespace nufuse::results
