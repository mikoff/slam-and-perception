/// @file results/optimizer.hpp
/// @brief Optimization and type-safe results unpacking.

#pragma once

#include "core/types.hpp"
#include "graph/builder.hpp"

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <vector>

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
    int num_keyframes = 0;
    int num_corrupted_gnss = 0;
};

/// @brief Run Levenberg-Marquardt optimization and unpack results.
OptimizedResults optimize(const graph::GtsamGraph& gtsam_graph,
                          const graph::FactorStorage& storage);

}  // namespace nufuse::results
