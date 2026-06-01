/// @file graph/builder.hpp
/// @brief Converts a FactorStorage into a GTSAM NonlinearFactorGraph + Values.

#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <unordered_map>

#include "core/pipeline_config.hpp"
#include "domain/calibration.hpp"
#include "graph/factor_storage.hpp"

namespace nufuse::graph {

/// @brief Result of GTSAM graph construction: factor graph + initial linearization point.
struct FactorGraphBundle {
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;
  int num_keyframes = 0;
  int num_corrupted = 0;

  /// @brief Maps Timestamp -> integer index for GTSAM symbol creation.
  std::unordered_map<uint64_t, int> stamp_to_index;
};

/// @brief Build a GTSAM NonlinearFactorGraph from a completed FactorStorage.
/// @param storage All factors and initial estimates.
/// @param extrinsics Sensor calibration for initial values of extrinsic variables.
/// @param config Pipeline configuration with noise parameters.
FactorGraphBundle buildGraph(const FactorStorage& storage,
                             const domain::ExtrinsicCalibration& extrinsics,
                             const core::PipelineConfig& config = {});

}  // namespace nufuse::graph
