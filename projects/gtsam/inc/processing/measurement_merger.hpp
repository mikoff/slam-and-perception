/// @file processing/measurement_merger.hpp
/// @brief Chronological interleaving of sensor measurements into FactorStorage.

#pragma once

#include <unordered_map>

#include "core/pipeline_config.hpp"
#include "domain/scene_data.hpp"
#include "graph/factor_storage.hpp"

namespace nufuse::processing {

/// @brief Merge all sensor measurements chronologically and produce a FactorStorage.
/// Handles IMU preintegration, GPS keyframe creation, and LiDAR keyframe creation.
graph::FactorStorage mergeMeasurements(const domain::SceneData& scene,
                                       const std::unordered_map<uint64_t, gtsam::Pose3>& lidar_rel,
                                       const core::PipelineConfig& pipeline_cfg, bool init_from_gt);

}  // namespace nufuse::processing
