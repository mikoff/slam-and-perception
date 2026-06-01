/// @file pipeline/pipeline.hpp
/// @brief High-level pipeline orchestrator for the NuFuse SLAM system.

#pragma once

#include "core/pipeline_config.hpp"
#include "domain/scene_data.hpp"
#include "graph/factor_storage.hpp"
#include "results/optimizer.hpp"

namespace nufuse::pipeline {

/// @brief Combined output of the NuFuse pipeline.
struct PipelineOutput {
  results::OptimizedResults results;
  graph::FactorStorage storage;
};

/// @brief Run the full NuFuse optimization pipeline on a scene.
/// @param scene Loaded sensor data.
/// @param config Pipeline configuration (noise models, enables, etc.).
/// @param init_from_gt If true, initialize first keyframe from ground truth position.
/// @return Optimized results and factor storage.
PipelineOutput run(const domain::SceneData& scene, const core::PipelineConfig& config = {},
                   bool init_from_gt = false);

}  // namespace nufuse::pipeline
