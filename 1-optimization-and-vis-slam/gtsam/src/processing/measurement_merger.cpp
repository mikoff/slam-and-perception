/// @file processing/measurement_merger.cpp
/// @brief Implementation of chronological measurement merging.

#include "processing/measurement_merger.hpp"

#include "processing/bias_estimator.hpp"
#include "processing/gps_corruptor.hpp"
#include "processing/measurement_processor.hpp"

namespace nufuse::processing {

graph::FactorStorage mergeMeasurements(const domain::SceneData& scene,
                                       const std::unordered_map<uint64_t, gtsam::Pose3>& lidar_rel,
                                       const core::PipelineConfig& pipeline_cfg,
                                       bool init_from_gt) {
  ProcessorConfig config;
  config.init_from_gt = init_from_gt;
  config.initial_bias = estimateInitialBias(scene.imu, scene.odom);
  config.imu = pipeline_cfg.imu;
  config.keyframe_interval_s = pipeline_cfg.imu_keyframe_interval_s;
  MeasurementProcessor processor(config, scene.odom);

  // Pre-corrupt GNSS only if corruption testing is enabled in config
  std::vector<CorruptedGnssFix> corrupted_gnss;
  if (pipeline_cfg.gps_corruption.enable) {
    GpsCorruptorConfig corrupt_cfg;
    corrupt_cfg.spike_metres = pipeline_cfg.gps_corruption.spike_metres;
    corrupt_cfg.spike_fraction = pipeline_cfg.gps_corruption.spike_fraction;
    corrupt_cfg.rng_seed = pipeline_cfg.gps_corruption.rng_seed;
    corrupted_gnss = corruptGnss(scene.gnss, corrupt_cfg);
  } else {
    corrupted_gnss.reserve(scene.gnss.size());
    for (const auto& fix : scene.gnss) {
      corrupted_gnss.push_back({fix, false});
    }
  }

  auto gnss_it = corrupted_gnss.cbegin();
  auto imu_it = scene.imu.cbegin();
  auto lidar_it = scene.lidar.cbegin();

  int gps_count = 0;

  while (gnss_it != corrupted_gnss.cend() || imu_it != scene.imu.cend() ||
         lidar_it != scene.lidar.cend()) {
    const uint64_t gnss_stamp =
        gnss_it != corrupted_gnss.cend() ? gnss_it->fix.stamp.value() : UINT64_MAX;
    const uint64_t imu_stamp = imu_it != scene.imu.cend() ? imu_it->stamp.value() : UINT64_MAX;
    const uint64_t lidar_stamp =
        lidar_it != scene.lidar.cend() ? lidar_it->stamp.value() : UINT64_MAX;

    if (imu_stamp <= gnss_stamp && imu_stamp <= lidar_stamp) {
      processor.addImu(*imu_it++);
    } else if (gnss_stamp <= lidar_stamp) {
      bool gps_active = pipeline_cfg.enable_gps;
      if (gps_active && pipeline_cfg.gps_max_measurements > 0 &&
          gps_count >= pipeline_cfg.gps_max_measurements) {
        gps_active = false;
      }
      if (gps_active) {
        processor.addGnss(gnss_it->fix, gnss_it->corrupted);
        ++gps_count;
      } else {
        if (gps_count == 0) {
          processor.addGnss(gnss_it->fix, gnss_it->corrupted);
          ++gps_count;
        }
      }
      ++gnss_it;
    } else {
      if (pipeline_cfg.enable_lidar) {
        const auto stamp = lidar_it->stamp;
        auto rel_it = lidar_rel.find(stamp.value());
        processor.addLidar(stamp, rel_it != lidar_rel.end()
                                      ? std::optional<gtsam::Pose3>(rel_it->second)
                                      : std::nullopt);
      }
      ++lidar_it;
    }
  }

  return std::move(processor).finalize();
}

}  // namespace nufuse::processing
