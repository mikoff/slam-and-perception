/// @file results/optimizer.cpp
/// @brief Implementation of optimization and results unpacking.

#include "results/optimizer.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <iostream>

#include "core/gtsam_boundary.hpp"

namespace nufuse::results {

using namespace gtsam;
using symbol_shorthand::B;
using symbol_shorthand::V;
using symbol_shorthand::X;

static const core::LidarExtrinsicSymbol kLidarExtrinsicSym('L', 0);
static const core::OdomScaleSymbol kOdomScaleSym('S', 0);

// ─── LM optimization ────────────────────────────────────────────────────────

static Values runLM(const NonlinearFactorGraph& graph, const Values& initial) {
  LevenbergMarquardtParams params;
  params.setVerbosityLM("SUMMARY");
  LevenbergMarquardtOptimizer optimizer(graph, initial, params);
  return optimizer.optimize();
}

// ─── GNC optimization (graduated non-convexity) ─────────────────────────────

static Values runGNC(const NonlinearFactorGraph& graph, const Values& initial,
                     const core::PipelineConfig::GncConfig& gnc_cfg) {
  using GncLMParams = GncParams<LevenbergMarquardtParams>;

  LevenbergMarquardtParams lm_params;
  lm_params.setVerbosityLM("SUMMARY");

  GncLMParams gnc_params(lm_params);
  gnc_params.setLossType(GncLossType::TLS);
  gnc_params.setMuStep(gnc_cfg.mu_step);
  gnc_params.setRelativeCostTol(gnc_cfg.relative_cost_tol);
  gnc_params.setWeightsTol(gnc_cfg.weights_tol);
  gnc_params.setVerbosityGNC(GncLMParams::Verbosity::SUMMARY);

  // Mark IMU/prior/NHC/fwd-vel factors as known inliers (only reweight
  // GPS/LiDAR/Radar perception factors). GNC will skip non-noise-model
  // factors automatically when allowNonNoiseModelFactors is set.
  gnc_params.setAllowNonNoiseModelFactors(true);

  GncOptimizer<GncLMParams> optimizer(graph, initial, gnc_params);
  return optimizer.optimize();
}

// ─── Covariance extraction ───────────────────────────────────────────────────

static void extractPosesAndVelocities(const Values& values, const Marginals& marginals,
                                      const graph::FactorStorage& storage,
                                      OptimizedResults& results) {
  for (int i = 0; i < static_cast<int>(storage.estimates.size()); ++i) {
    const auto pose_sym = core::poseSymbol(i);
    const auto vel_sym = core::velocitySymbol(i);
    const auto bias_sym = core::biasSymbol(i);

    OptimizedPose pose_entry;
    pose_entry.stamp = storage.estimates[i].stamp;
    pose_entry.pose = core::extractPose(values, pose_sym);
    pose_entry.covariance = marginals.marginalCovariance(pose_sym.key());
    results.poses.push_back(pose_entry);

    OptimizedVelocity velocity_entry;
    velocity_entry.stamp = storage.estimates[i].stamp;
    velocity_entry.velocity = core::extractVelocity(values, vel_sym);
    velocity_entry.covariance = marginals.marginalCovariance(vel_sym.key());
    results.velocities.push_back(velocity_entry);

    OptimizedBias bias_entry;
    bias_entry.stamp = storage.estimates[i].stamp;
    bias_entry.bias = values.at<imuBias::ConstantBias>(bias_sym.key());
    bias_entry.covariance = marginals.marginalCovariance(bias_sym.key());
    results.biases.push_back(bias_entry);
  }
}

static void extractLidarExtrinsics(const Values& values, const Marginals& marginals,
                                   OptimizedResults& results) {
  if (values.exists(kLidarExtrinsicSym.key())) {
    results.body_from_lidar = core::extractLidarExtrinsic(values, kLidarExtrinsicSym);
    results.lidar_extrinsics_covariance = marginals.marginalCovariance(kLidarExtrinsicSym.key());
  } else {
    results.body_from_lidar = core::BodyFromLidar(Pose3::Identity());
    results.lidar_extrinsics_covariance.setZero();
  }
}

static void extractRadarExtrinsics(const Values& values, OptimizedResults& results) {
  // Check R(0) through R(4) for radar extrinsic keys
  for (int i = 0; i < 5; ++i) {
    const auto id = static_cast<core::RadarSensorId>(i);
    const auto sym = core::radarSymbol(id);
    if (values.exists(sym.key())) {
      results.radar_extrinsics[id] = core::extractRadarExtrinsicRaw(values, sym);
    }
  }
}

static void extractOdomScale(const Values& values, OptimizedResults& results) {
  if (values.exists(kOdomScaleSym.key())) {
    results.odom_scale_delta = core::OdomScaleDelta(values.at<double>(kOdomScaleSym.key()));
  }
}

// ─── Public interface ────────────────────────────────────────────────────────

OptimizedResults optimize(const graph::FactorGraphBundle& gtsam_graph,
                          const graph::FactorStorage& storage, const core::PipelineConfig& config) {
  Values values;
  switch (config.optimizer) {
    case core::OptimizerType::GNC_LM:
      values = runGNC(gtsam_graph.graph, gtsam_graph.initial, config.gnc);
      break;
    case core::OptimizerType::LM:
    default:
      values = runLM(gtsam_graph.graph, gtsam_graph.initial);
      break;
  }

  OptimizedResults results;
  results.num_keyframes = gtsam_graph.num_keyframes;
  results.num_corrupted_gnss = gtsam_graph.num_corrupted;

  // Extract results: first try with covariances, fall back to values-only
  auto extractWithoutCovariance = [&]() {
    for (int i = 0; i < static_cast<int>(storage.estimates.size()); ++i) {
      const auto pose_sym = core::poseSymbol(i);
      const auto vel_sym = core::velocitySymbol(i);
      const auto bias_sym = core::biasSymbol(i);

      if (!values.exists(pose_sym.key())) continue;

      OptimizedPose pose_entry;
      pose_entry.stamp = storage.estimates[i].stamp;
      pose_entry.pose = core::extractPose(values, pose_sym);
      pose_entry.covariance.setZero();
      results.poses.push_back(pose_entry);

      OptimizedVelocity velocity_entry;
      velocity_entry.stamp = storage.estimates[i].stamp;
      velocity_entry.velocity = core::extractVelocity(values, vel_sym);
      velocity_entry.covariance.setZero();
      results.velocities.push_back(velocity_entry);

      OptimizedBias bias_entry;
      bias_entry.stamp = storage.estimates[i].stamp;
      bias_entry.bias = values.at<imuBias::ConstantBias>(bias_sym.key());
      bias_entry.covariance.setZero();
      results.biases.push_back(bias_entry);
    }
    if (values.exists(kLidarExtrinsicSym.key())) {
      results.body_from_lidar = core::extractLidarExtrinsic(values, kLidarExtrinsicSym);
    }
    results.lidar_extrinsics_covariance.setZero();
  };

  try {
    Marginals marginals(gtsam_graph.graph, values);
    extractPosesAndVelocities(values, marginals, storage, results);
    extractLidarExtrinsics(values, marginals, results);
  } catch (const IndeterminantLinearSystemException&) {
    extractWithoutCovariance();
    std::cerr << "Warning: Marginals computation failed (ill-conditioned system)\n";
  } catch (const ValuesKeyDoesNotExist& e) {
    extractWithoutCovariance();
    std::cerr << "Warning: Marginals computation failed (missing key: " << e.what() << ")\n";
  }

  extractRadarExtrinsics(values, results);
  extractOdomScale(values, results);

  return results;
}

}  // namespace nufuse::results
