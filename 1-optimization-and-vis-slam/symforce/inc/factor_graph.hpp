#pragma once

/// @file factor_graph.hpp
/// @brief Factor-graph orchestrator: owns storage, runs optimizer, exports JSON.

#include <string>

#include "factor_graph_storage.hpp"

/// @brief Factor-graph orchestrator, templated on geometry policy G.
///
/// Owns a CFactorGraphStorage, exposes optimization and JSON export.
template <typename G>
class CFactorGraph {
 public:
  CFactorGraph() = default;

  /// @brief Mutable access to the underlying storage.
  CFactorGraphStorage<G>& storage();

  /// @brief Const access to the underlying storage.
  const CFactorGraphStorage<G>& storage() const;

  /// @brief Optimizer configuration.
  struct OptimizeParams {
    int iterations = 200;
    double early_exit_min_reduction = 1e-8;
    bool verbose = true;
  };

  /// @brief Optimization result summary.
  struct OptimizeResult {
    double initial_error = 0.0;
    double final_error = 0.0;
    int num_iterations = 0;
  };

  /// @brief Run optimization with default parameters.
  OptimizeResult optimize();

  /// @brief Run optimization with custom parameters.
  OptimizeResult optimize(const OptimizeParams& p);

  /// @brief Export results to JSON (zero-initialized stats).
  void saveJson(const std::string& path) const;

  /// @brief Export results to JSON with optimization stats.
  void saveJson(const std::string& path,
                const OptimizeResult& opt_result) const;

 private:
  CFactorGraphStorage<G> storage_;
};

#include "factor_graph.inl"
