/// @file graph/builder.cpp
/// @brief Implementation of graph construction.

#include "graph/builder.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <set>

#include "core/gtsam_boundary.hpp"
#include "factor/forward_velocity.hpp"
#include "factor/lidar_extrinsics.hpp"
#include "factor/non_holonomic.hpp"
#include "factor/spatiotemporal_radar.hpp"

namespace nufuse::graph {

using namespace gtsam;
using symbol_shorthand::B;
using symbol_shorthand::V;
using symbol_shorthand::X;

static const Symbol kLidarExtrinsicKey('L', 0);
static const Symbol kOdomScaleKey('S', 0);

// ─── Noise models ────────────────────────────────────────────────────────────

struct NoiseModels {
  SharedNoiseModel pose;
  SharedNoiseModel velocity;
  SharedNoiseModel bias;
  SharedNoiseModel gps;
  SharedNoiseModel lidar;
  SharedNoiseModel radar;
  SharedNoiseModel nhc;
  SharedNoiseModel forward_velocity;
};

/// @brief Optionally wrap a base noise model in a robust kernel.
/// When GNC is the optimizer, robust wrapping is skipped (GNC manages outlier weights directly).
static SharedNoiseModel maybeRobust(const SharedNoiseModel& base, double threshold,
                                    const core::PipelineConfig& cfg) {
  if (cfg.optimizer == core::OptimizerType::GNC_LM) {
    return base;  // GNC handles robustification — no kernel wrapping needed
  }
  noiseModel::mEstimator::Base::shared_ptr kernel;
  switch (cfg.robust_kernel) {
    case core::RobustKernelType::Cauchy:
      kernel = noiseModel::mEstimator::Cauchy::Create(threshold);
      break;
    case core::RobustKernelType::Huber:
    default:
      kernel = noiseModel::mEstimator::Huber::Create(threshold);
      break;
  }
  return noiseModel::Robust::Create(kernel, base);
}

static NoiseModels makeNoiseModels(bool has_heading_observability,
                                   const core::PipelineConfig& cfg) {
  NoiseModels models;
  const auto& pr = cfg.prior;
  models.pose = noiseModel::Diagonal::Sigmas((Vector(6) << pr.pose_rot_sigma, pr.pose_rot_sigma,
                                              pr.pose_rot_sigma, pr.pose_trans_sigma,
                                              pr.pose_trans_sigma, pr.pose_trans_sigma)
                                                 .finished());
  models.velocity = noiseModel::Isotropic::Sigma(3, pr.velocity_sigma);

  const double gyro_sigma = has_heading_observability ? pr.bias_gyro_sigma_obs : pr.bias_gyro_sigma;
  models.bias =
      noiseModel::Diagonal::Sigmas((Vector(6) << pr.bias_accel_sigma, pr.bias_accel_sigma,
                                    pr.bias_accel_sigma, gyro_sigma, gyro_sigma, gyro_sigma)
                                       .finished());

  auto gps_base = noiseModel::Isotropic::Sigma(3, cfg.gps.sigma);
  models.gps = maybeRobust(gps_base, cfg.gps.robust_threshold, cfg);

  auto lidar_base = noiseModel::Diagonal::Sigmas(
      (Vector(6) << cfg.lidar.odom_rot_sigma, cfg.lidar.odom_rot_sigma, cfg.lidar.odom_rot_sigma,
       cfg.lidar.odom_trans_sigma, cfg.lidar.odom_trans_sigma, cfg.lidar.odom_trans_sigma)
          .finished());
  models.lidar = maybeRobust(lidar_base, cfg.lidar.robust_threshold, cfg);

  auto radar_base = noiseModel::Isotropic::Sigma(1, cfg.radar.velocity_sigma);
  models.radar = maybeRobust(radar_base, cfg.radar.robust_threshold, cfg);

  models.nhc = noiseModel::Diagonal::Sigmas(
      (Vector(2) << cfg.nhc.lateral_sigma, cfg.nhc.vertical_sigma).finished());

  models.forward_velocity = noiseModel::Isotropic::Sigma(1, cfg.forward_velocity.sigma);
  return models;
}

// ─── Timestamp to index mapping ─────────────────────────────────────────────

static std::unordered_map<uint64_t, int> buildStampIndex(const FactorStorage& storage) {
  std::unordered_map<uint64_t, int> stamp_index;
  for (int i = 0; i < static_cast<int>(storage.estimates.size()); ++i) {
    stamp_index[storage.estimates[i].stamp.value()] = i;
  }
  return stamp_index;
}

// ─── Factor insertion helpers ────────────────────────────────────────────────

static void addPriorFactors(NonlinearFactorGraph& graph, const PriorFactor& prior, int idx,
                            const NoiseModels& noise) {
  graph.addPrior<Pose3>(X(idx), prior.pose.value(), noise.pose);
  graph.addPrior<Vector3>(V(idx), prior.velocity.value(), noise.velocity);
  graph.addPrior<imuBias::ConstantBias>(B(idx), prior.bias, noise.bias);
  graph.add(GPSFactor(X(idx), prior.gps_position.value(), noise.gps));
}

static void addImuFactors(NonlinearFactorGraph& graph, const std::vector<StoredImuFactor>& factors,
                          const std::unordered_map<uint64_t, int>& stamp_idx) {
  for (const auto& factor : factors) {
    const int from_idx = stamp_idx.at(factor.stamp_from.value());
    const int to_idx = stamp_idx.at(factor.stamp_to.value());
    graph.add(CombinedImuFactor(X(from_idx), V(from_idx), X(to_idx), V(to_idx), B(from_idx),
                                B(to_idx), factor.pim));
  }
}

static void addGpsFactors(NonlinearFactorGraph& graph, const std::vector<StoredGpsFactor>& factors,
                          const std::unordered_map<uint64_t, int>& stamp_idx,
                          const NoiseModels& noise) {
  for (const auto& factor : factors) {
    const int idx = stamp_idx.at(factor.stamp.value());
    graph.add(GPSFactor(X(idx), factor.position_enu.value(), noise.gps));
  }
}

static void addLidarFactors(NonlinearFactorGraph& graph,
                            const std::vector<StoredLidarFactor>& factors,
                            const std::unordered_map<uint64_t, int>& stamp_idx,
                            const NoiseModels& noise) {
  for (const auto& factor : factors) {
    const int from_idx = stamp_idx.at(factor.stamp_from.value());
    const int to_idx = stamp_idx.at(factor.stamp_to.value());
    graph.add(factor::LidarExtrinsicsFactor(X(from_idx), X(to_idx), kLidarExtrinsicKey,
                                            factor.relative_pose.value(), noise.lidar));
  }
}

static void addRadarFactors(NonlinearFactorGraph& graph,
                            const std::vector<StoredRadarFactor>& factors,
                            const std::unordered_map<uint64_t, int>& stamp_idx,
                            const NoiseModels& noise, std::set<Key>& radar_extrinsic_keys) {
  for (const auto& factor : factors) {
    auto it = stamp_idx.find(factor.stamp.value());
    if (it == stamp_idx.end()) continue;
    const int idx = it->second;

    for (const auto& meas : factor.measurements) {
      graph.add(factor::SpatiotemporalRadarFactor(X(idx), V(idx), factor.extrinsic_key,
                                                  meas.v_radial, meas.bearing.value(),
                                                  meas.omega_B.value(), noise.radar));
    }
    radar_extrinsic_keys.insert(factor.extrinsic_key);
  }
}

static void addNhcFactors(NonlinearFactorGraph& graph, const std::vector<StoredNhcFactor>& factors,
                          const std::unordered_map<uint64_t, int>& stamp_idx,
                          const NoiseModels& noise, const core::NhcConfig& nhc_cfg) {
  const Vector3 lever_arm(nhc_cfg.lever_arm_x, nhc_cfg.lever_arm_y, nhc_cfg.lever_arm_z);
  for (const auto& f : factors) {
    auto it = stamp_idx.find(f.stamp.value());
    if (it == stamp_idx.end()) continue;
    const int idx = it->second;
    graph.add(factor::NonHolonomicFactor(X(idx), V(idx), f.omega_B.value(), lever_arm, noise.nhc));
  }
}

static void addForwardVelocityFactors(NonlinearFactorGraph& graph,
                                      const std::vector<StoredForwardVelocityFactor>& factors,
                                      const std::unordered_map<uint64_t, int>& stamp_idx,
                                      const NoiseModels& noise, const core::NhcConfig& nhc_cfg) {
  const Vector3 lever_arm(nhc_cfg.lever_arm_x, nhc_cfg.lever_arm_y, nhc_cfg.lever_arm_z);
  for (const auto& f : factors) {
    auto it_from = stamp_idx.find(f.stamp_from.value());
    auto it_to = stamp_idx.find(f.stamp_to.value());
    if (it_from == stamp_idx.end() || it_to == stamp_idx.end()) continue;
    const int from_idx = it_from->second;
    const int to_idx = it_to->second;
    graph.add(factor::ForwardVelocityScaleFactor(X(from_idx), X(to_idx), kOdomScaleKey,
                                                 f.measured_v, f.dt, f.omega_B.value(), lever_arm,
                                                 noise.forward_velocity));
  }
}

static Pose3 getRadarCalibration(Key key, const domain::ExtrinsicCalibration& extrinsics) {
  const int idx = static_cast<int>(Symbol(key).index());
  return extrinsics.radarExtrinsic(idx);
}

static void insertInitialValues(Values& values, NonlinearFactorGraph& graph,
                                const std::vector<KeyframeEstimate>& estimates,
                                bool has_lidar_factors, bool has_fwd_vel_factors,
                                bool has_heading_obs, const std::set<Key>& radar_extrinsic_keys,
                                const domain::ExtrinsicCalibration& extrinsics,
                                const core::PipelineConfig& cfg) {
  if (has_lidar_factors) {
    const Pose3 lidar_calib = extrinsics.body_from_lidar_top
                                  ? extrinsics.body_from_lidar_top->value()
                                  : Pose3::Identity();
    values.insert(kLidarExtrinsicKey, lidar_calib);
    auto lidar_ext_prior_noise = noiseModel::Diagonal::Sigmas(
        (Vector(6) << cfg.lidar.extrinsic_prior_rot_sigma, cfg.lidar.extrinsic_prior_rot_sigma,
         cfg.lidar.extrinsic_prior_rot_sigma, cfg.lidar.extrinsic_prior_trans_sigma,
         cfg.lidar.extrinsic_prior_trans_sigma, cfg.lidar.extrinsic_prior_trans_sigma)
            .finished());
    graph.addPrior<Pose3>(kLidarExtrinsicKey, lidar_calib, lidar_ext_prior_noise);
  }

  auto radar_ext_prior_noise = noiseModel::Diagonal::Sigmas(
      (Vector(6) << cfg.radar.extrinsic_prior_rot_sigma, cfg.radar.extrinsic_prior_rot_sigma,
       cfg.radar.extrinsic_prior_yaw_sigma, cfg.radar.extrinsic_prior_trans_xy,
       cfg.radar.extrinsic_prior_trans_xy, cfg.radar.extrinsic_prior_trans_z)
          .finished());
  for (const auto& key : radar_extrinsic_keys) {
    const Pose3 calibrated = getRadarCalibration(key, extrinsics);
    values.insert(key, calibrated);
    graph.addPrior<Pose3>(key, calibrated, radar_ext_prior_noise);
  }

  if (has_fwd_vel_factors) {
    const double scale_init = has_heading_obs ? -0.03 : 0.0;
    const double scale_sigma = has_heading_obs ? cfg.forward_velocity.scale_prior_sigma : 0.01;
    values.insert(kOdomScaleKey, scale_init);
    auto scale_prior_noise = noiseModel::Isotropic::Sigma(1, scale_sigma);
    graph.addPrior<double>(kOdomScaleKey, scale_init, scale_prior_noise);
  }

  for (int i = 0; i < static_cast<int>(estimates.size()); ++i) {
    const auto& estimate = estimates[i];
    values.insert(X(i), estimate.pose.value());
    values.insert(V(i), estimate.velocity.value());
    values.insert(B(i), estimate.bias);
  }
}

// ─── Public interface ────────────────────────────────────────────────────────

FactorGraphBundle buildGraph(const FactorStorage& storage,
                             const domain::ExtrinsicCalibration& extrinsics,
                             const core::PipelineConfig& config) {
  const bool has_heading_obs = !storage.lidar_factors.empty();
  const auto noise = makeNoiseModels(has_heading_obs, config);
  FactorGraphBundle result;
  result.stamp_to_index = buildStampIndex(storage);

  if (storage.prior) {
    const int idx = result.stamp_to_index.at(storage.prior->stamp.value());
    addPriorFactors(result.graph, *storage.prior, idx, noise);
  }

  addImuFactors(result.graph, storage.imu_factors, result.stamp_to_index);
  addGpsFactors(result.graph, storage.gps_factors, result.stamp_to_index, noise);
  addLidarFactors(result.graph, storage.lidar_factors, result.stamp_to_index, noise);

  std::set<Key> radar_extrinsic_keys;
  addRadarFactors(result.graph, storage.radar_factors, result.stamp_to_index, noise,
                  radar_extrinsic_keys);
  addNhcFactors(result.graph, storage.nhc_factors, result.stamp_to_index, noise, config.nhc);
  addForwardVelocityFactors(result.graph, storage.forward_velocity_factors, result.stamp_to_index,
                            noise, config.nhc);

  const bool has_lidar = !storage.lidar_factors.empty();
  const bool has_fwd_vel = !storage.forward_velocity_factors.empty();
  insertInitialValues(result.initial, result.graph, storage.estimates, has_lidar, has_fwd_vel,
                      has_heading_obs, radar_extrinsic_keys, extrinsics, config);

  result.num_keyframes = static_cast<int>(storage.estimates.size()) - 1;
  result.num_corrupted = storage.num_corrupted;
  return result;
}

}  // namespace nufuse::graph
