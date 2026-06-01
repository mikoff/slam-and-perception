/// @file results/reporter.hpp
/// @brief Console reporting of optimization results.

#pragma once

#include "domain/scene_data.hpp"
#include "results/optimizer.hpp"

namespace nufuse::results {

/// @brief Print optimized trajectory (first/last 3 poses).
void printTrajectory(const OptimizedResults& res);

/// @brief Print final IMU bias estimate.
void printBias(const OptimizedResults& res);

/// @brief Print end-position error vs ground truth.
void printError(const OptimizedResults& res, const domain::SceneData& scene, bool init_from_gt);

/// @brief Print estimated vs ground truth LiDAR extrinsics.
void printLidarExtrinsics(const OptimizedResults& res, const domain::SceneData& scene);

/// @brief Print estimated vs ground truth radar extrinsics.
void printRadarExtrinsics(const OptimizedResults& res, const domain::SceneData& scene);

}  // namespace nufuse::results
