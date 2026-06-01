/// @file pipeline/pipeline.cpp
/// @brief Implementation of the NuFuse pipeline orchestrator.

#include "pipeline/pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <unordered_map>

#include "graph/builder.hpp"
#include "processing/forward_velocity_generator.hpp"
#include "processing/lidar_odometry.hpp"
#include "processing/measurement_merger.hpp"
#include "processing/nhc_generator.hpp"
#include "processing/radar_processor.hpp"

namespace nufuse::pipeline {

namespace {

/// @brief Gate a single LiDAR registration result against quality criteria.
bool passesQualityGate(const processing::LidarRelativePose& pose,
                       const core::LidarNoiseConfig& cfg) {
  if (!pose.converged) return false;
  if (pose.num_inliers < cfg.min_inliers) return false;
  if (pose.error > cfg.max_registration_error) return false;

  const double translation_norm = pose.T_body.value().translation().norm();
  if (translation_norm > cfg.max_translation_m) return false;

  return true;
}

/// @brief Build a lookup from timestamp → IMU-preintegrated body delta for consistency checking.
/// Uses consecutive keyframe IMU factors: the preintegrated translation gives expected
/// displacement.
std::unordered_map<uint64_t, gtsam::Pose3> buildImuPredictions(
    const graph::FactorStorage& storage) {
  std::unordered_map<uint64_t, gtsam::Pose3> predictions;
  for (const auto& imu_factor : storage.imu_factors) {
    // Predicted relative pose from IMU preintegration (ignoring velocity/bias corrections
    // which are small for the purpose of outlier gating)
    predictions[imu_factor.stamp_to.value()] =
        imu_factor.pim.predict(gtsam::NavState(), gtsam::imuBias::ConstantBias()).pose();
  }
  return predictions;
}

}  // anonymous namespace

results::OptimizedResults run(const domain::SceneData& scene, const core::PipelineConfig& config,
                              bool init_from_gt) {
  // 1. LiDAR odometry with quality gating
  std::unordered_map<uint64_t, gtsam::Pose3> lidar_rel;
  std::vector<processing::LidarRelativePose> lidar_poses_raw;
  if (config.enable_lidar && !scene.lidar.empty() && scene.extrinsics.body_from_lidar_top) {
    lidar_poses_raw =
        processing::computeLidarOdometry(scene.lidar, *scene.extrinsics.body_from_lidar_top);
    for (const auto& pose : lidar_poses_raw) {
      if (passesQualityGate(pose, config.lidar)) {
        lidar_rel[pose.stamp_to.value()] = pose.T_lidar.value();
      }
    }
  }

  // 2. Merge measurements (IMU + GPS + LiDAR)
  auto storage = processing::mergeMeasurements(scene, lidar_rel, config, init_from_gt);

  // 2b. IMU-consistency check: remove LiDAR factors whose body-frame displacement
  //     deviates too much from the IMU-preintegrated prediction.
  if (config.lidar.imu_consistency_threshold > 0.0 && !storage.lidar_factors.empty()) {
    auto imu_predictions = buildImuPredictions(storage);
    auto& lidar_factors = storage.lidar_factors;
    lidar_factors.erase(std::remove_if(lidar_factors.begin(), lidar_factors.end(),
                                       [&](const graph::StoredLidarFactor& f) {
                                         auto it = imu_predictions.find(f.stamp_to.value());
                                         if (it == imu_predictions.end())
                                           return false;  // no IMU reference, keep
                                         const double imu_trans = it->second.translation().norm();
                                         const double lidar_trans =
                                             f.relative_pose.value().translation().norm();
                                         return std::abs(lidar_trans - imu_trans) >
                                                config.lidar.imu_consistency_threshold;
                                       }),
                        lidar_factors.end());
  }

  // 3. Radar processing
  if (config.enable_radar && !scene.radar.empty()) {
    auto radar_cfg = processing::RadarProcessorConfig::fromPipelineConfig(config);
    storage.radar_factors =
        processing::processRadarScans(scene.radar, scene.extrinsics, scene.odom, radar_cfg);
  }

  // 4. NHC + forward velocity
  if (config.enable_nhc) {
    storage.nhc_factors = processing::generateNhcFactors(storage, scene.odom);
    if (config.enable_fwdvel) {
      storage.forward_velocity_factors =
          processing::generateForwardVelocityFactors(storage, scene.odom);
    }
  }

  // 5. Build graph & optimize
  auto graph_bundle = graph::buildGraph(storage, scene.extrinsics, config);
  return results::optimize(graph_bundle, storage, config);
}

}  // namespace nufuse::pipeline
