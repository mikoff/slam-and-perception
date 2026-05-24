/// @file results/optimizer.cpp
/// @brief Implementation of optimization and results unpacking.

#include "results/optimizer.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

namespace nufuse::results {

using namespace gtsam;
using symbol_shorthand::B;
using symbol_shorthand::V;
using symbol_shorthand::X;

static const Symbol kLidarExtrinsicKey('L', 0);

// ─── LM optimization ────────────────────────────────────────────────────────

static Values runLM(const NonlinearFactorGraph& graph, const Values& initial) {
  LevenbergMarquardtParams params;
  params.setVerbosityLM("SUMMARY");
  LevenbergMarquardtOptimizer optimizer(graph, initial, params);
  return optimizer.optimize();
}

// ─── Covariance extraction ───────────────────────────────────────────────────

static void extractPosesAndVelocities(const Values& values, const Marginals& marginals,
                                      const graph::FactorStorage& storage,
                                      OptimizedResults& results) {
  for (int i = 0; i < static_cast<int>(storage.estimates.size()); ++i) {
    OptimizedPose pose_entry;
    pose_entry.stamp = storage.estimates[i].stamp;
    pose_entry.pose = core::BodyInEnu(values.at<Pose3>(X(i)));
    pose_entry.covariance = marginals.marginalCovariance(X(i));
    results.poses.push_back(pose_entry);

    OptimizedVelocity velocity_entry;
    velocity_entry.stamp = storage.estimates[i].stamp;
    velocity_entry.velocity = core::EnuVelocity(values.at<Vector3>(V(i)));
    velocity_entry.covariance = marginals.marginalCovariance(V(i));
    results.velocities.push_back(velocity_entry);

    OptimizedBias bias_entry;
    bias_entry.stamp = storage.estimates[i].stamp;
    bias_entry.bias = values.at<imuBias::ConstantBias>(B(i));
    bias_entry.covariance = marginals.marginalCovariance(B(i));
    results.biases.push_back(bias_entry);
  }
}

static void extractLidarExtrinsics(const Values& values, const Marginals& marginals,
                                   OptimizedResults& results) {
  if (values.exists(kLidarExtrinsicKey)) {
    results.body_from_lidar = core::BodyFromLidar(values.at<Pose3>(kLidarExtrinsicKey));
    results.lidar_extrinsics_covariance = marginals.marginalCovariance(kLidarExtrinsicKey);
  } else {
    results.body_from_lidar = core::BodyFromLidar(Pose3::Identity());
    results.lidar_extrinsics_covariance.setZero();
  }
}

// ─── Public interface ────────────────────────────────────────────────────────

OptimizedResults optimize(const graph::GtsamGraph& gtsam_graph,
                          const graph::FactorStorage& storage) {
  Values values = runLM(gtsam_graph.graph, gtsam_graph.initial);
  Marginals marginals(gtsam_graph.graph, values);

  OptimizedResults results;
  results.num_keyframes = gtsam_graph.num_keyframes;
  results.num_corrupted_gnss = gtsam_graph.num_corrupted;

  extractPosesAndVelocities(values, marginals, storage, results);
  extractLidarExtrinsics(values, marginals, results);

  return results;
}

}  // namespace nufuse::results
