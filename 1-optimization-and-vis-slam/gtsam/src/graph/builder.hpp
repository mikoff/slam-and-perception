/// @file graph/builder.hpp
/// @brief Converts a FactorStorage into a GTSAM NonlinearFactorGraph + Values.

#pragma once

#include "graph/factor_storage.hpp"

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <unordered_map>

namespace nufuse::graph {

/// @brief Result of GTSAM graph construction.
struct GtsamGraph {
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    int num_keyframes = 0;
    int num_corrupted = 0;

    /// @brief Maps Timestamp -> integer index for GTSAM symbol creation.
    std::unordered_map<uint64_t, int> stamp_to_index;
};

/// @brief Build a GTSAM NonlinearFactorGraph from a completed FactorStorage.
GtsamGraph buildGraph(const FactorStorage& storage);

}  // namespace nufuse::graph
