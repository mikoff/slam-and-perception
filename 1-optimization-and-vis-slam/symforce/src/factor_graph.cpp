/// @file factor_graph.cpp
/// @brief CFactorGraph — optimize + saveJson + explicit instantiations.

#include "factor_graph.hpp"

#include <fstream>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
#include <symforce/opt/key.h>
#include <symforce/opt/optimizer.h>

template <typename G>
typename CFactorGraph<G>::OptimizeResult CFactorGraph<G>::optimize() {
  return optimize(OptimizeParams{});
}

template <typename G>
typename CFactorGraph<G>::OptimizeResult
CFactorGraph<G>::optimize(const OptimizeParams& p) {
  auto params = sym::DefaultOptimizerParams();
  params.verbose = p.verbose;
  params.iterations = p.iterations;
  params.early_exit_min_reduction = p.early_exit_min_reduction;

  sym::Optimizer<double> optimizer(params, storage_.factors());
  const auto stats = optimizer.Optimize(storage_.values());

  OptimizeResult res;
  res.initial_error = stats.iterations.front().new_error;
  res.final_error = stats.iterations[stats.best_index].new_error;
  res.num_iterations = static_cast<int>(stats.iterations.size());

  spdlog::info("Optimization done: {} iterations, error {:.2f} -> {:.2f}",
               res.num_iterations, res.initial_error, res.final_error);
  return res;
}

template <typename G>
void CFactorGraph<G>::saveJson(const std::string& path) const {
  saveJson(path, OptimizeResult{});
}

template <typename G>
void CFactorGraph<G>::saveJson(
    const std::string& path, const OptimizeResult& opt_result) const {
  using json = nlohmann::json;
  json result;
  const auto& vals = storage_.values();

  json poses_arr = json::array();
  int n = storage_.numPoses();
  for (int i = 0; i < n; ++i) {
    if (!storage_.hasPose(i)) continue;
    auto pose = vals.template At<typename G::Pose>(sym::Key('P', i));
    poses_arr.push_back(G::poseToJson(i, pose));
  }
  result["poses"] = poses_arr;

  json lm_arr = json::array();
  for (int lm_id : storage_.landmarkIds()) {
    auto pos =
        vals.template At<typename G::Position>(sym::Key('L', lm_id));
    lm_arr.push_back(G::landmarkToJson(lm_id, pos));
  }
  result["landmarks"] = lm_arr;

  result["num_poses"] = n;
  result["num_landmarks"] =
      static_cast<int>(storage_.landmarkIds().size());
  result["num_factors"] = storage_.numFactors();
  result["initial_error"] = opt_result.initial_error;
  result["final_error"] = opt_result.final_error;
  result["num_iterations"] = opt_result.num_iterations;

  std::ofstream out(path);
  if (!out.is_open()) {
    throw std::runtime_error("Cannot open output file: " + path);
  }
  out << result.dump(2);
  spdlog::info("Results saved to {}", path);
}

// ---- Explicit instantiations --------------------------------------------
template class CFactorGraph<SE2>;
template class CFactorGraph<SE3>;
