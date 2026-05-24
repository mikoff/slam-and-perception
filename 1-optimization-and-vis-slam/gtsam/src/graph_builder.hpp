/// @file graph_builder.hpp
/// @brief Converts a FactorStorage into a GTSAM NonlinearFactorGraph + Values.
///
/// Pure function: reads the storage, iterates through all stored factors, and
/// constructs the GTSAM-specific graph.  No measurement processing logic.

#pragma once

#include "factor_storage.hpp"

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

/// @brief Result of GTSAM graph construction.
struct GtsamGraph {
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    int num_keyframes = 0;
    int num_corrupted = 0;
};

/// @brief Build a GTSAM NonlinearFactorGraph from a completed FactorStorage.
///
/// Iterates through all stored factors and initial estimates, converting them
/// to their GTSAM equivalents with appropriate noise models.
GtsamGraph buildGtsamGraph(const FactorStorage& storage);
