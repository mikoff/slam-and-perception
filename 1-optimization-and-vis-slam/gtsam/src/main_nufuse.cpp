/// @file main_nufuse.cpp
/// @brief CLI entry point for the NuFuse multi-sensor SLAM pipeline.
///
/// Pipeline:
///   1. Load MCAP -> typed sensor measurements.
///   2. Compute LiDAR GICP odometry -> body-frame relative poses.
///   3. Merge measurements chronologically -> FactorStorage.
///   4. Build GTSAM factor graph -> optimize -> type-safe results.
///   5. Report and export.

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <optional>
#include <unordered_map>

#include "core/config_loader.hpp"
#include "core/pipeline_config.hpp"
#include "domain/scene_data.hpp"
#include "graph/builder.hpp"
#include "io/mcap_loader.hpp"
#include "io/mcap_writer.hpp"
#include "processing/forward_velocity_generator.hpp"
#include "processing/lidar_odometry.hpp"
#include "processing/measurement_merger.hpp"
#include "processing/nhc_generator.hpp"
#include "processing/radar_processor.hpp"
#include "results/optimizer.hpp"
#include "results/reporter.hpp"

namespace {

bool validateScene(const nufuse::domain::SceneData& scene) {
  if (scene.imu.empty() || scene.gnss.size() < 2 || scene.odom.empty()) {
    std::cerr << "Insufficient data (need IMU, >=2 GNSS, odom).\n";
    return false;
  }
  if (!scene.extrinsics.body_from_lidar_top) {
    std::cerr << "Missing body_from_lidar_top extrinsic!\n";
    return false;
  }
  return true;
}

std::unordered_map<uint64_t, gtsam::Pose3> buildLidarRelMap(
    const std::vector<nufuse::processing::LidarRelativePose>& lidar_poses) {
  std::unordered_map<uint64_t, gtsam::Pose3> lidar_rel_map;
  for (const auto& pose : lidar_poses) {
    if (pose.converged) {
      lidar_rel_map[pose.stamp_to.value()] = pose.T_lidar.value();
    }
  }
  return lidar_rel_map;
}

}  // anonymous namespace

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0]
              << " <path-to-mcap> [--output <path>] [--config <json>] [--no-gps] "
                 "[--no-lidar] [--no-radar] [--no-nhc] [--no-fwdvel] [--init-from-gt] "
                 "[--gps-max N]\n";
    return EXIT_FAILURE;
  }

  const std::filesystem::path mcap_path(argv[1]);
  if (!std::filesystem::exists(mcap_path)) {
    std::cerr << "File not found: " << mcap_path << "\n";
    return EXIT_FAILURE;
  }

  // Parse CLI arguments
  std::optional<std::filesystem::path> output_path;
  std::optional<std::filesystem::path> config_path;
  nufuse::core::PipelineConfig pipeline_cfg;
  bool init_from_gt = false;

  for (int i = 2; i < argc; ++i) {
    const std::string_view arg(argv[i]);
    if (arg == "--output" && i + 1 < argc)
      output_path = argv[++i];
    else if (arg == "--config" && i + 1 < argc)
      config_path = argv[++i];
    else if (arg == "--no-gps")
      pipeline_cfg.enable_gps = false;
    else if (arg == "--no-lidar")
      pipeline_cfg.enable_lidar = false;
    else if (arg == "--no-radar")
      pipeline_cfg.enable_radar = false;
    else if (arg == "--no-nhc")
      pipeline_cfg.enable_nhc = false;
    else if (arg == "--no-fwdvel")
      pipeline_cfg.enable_fwdvel = false;
    else if (arg == "--init-from-gt")
      init_from_gt = true;
    else if (arg == "--gps-max" && i + 1 < argc)
      pipeline_cfg.gps_max_measurements = std::atoi(argv[++i]);
  }

  // Load JSON config if provided (overrides compiled defaults, CLI flags override JSON)
  if (config_path) {
    pipeline_cfg = nufuse::core::loadConfig(*config_path);
    // Re-apply CLI overrides that were already parsed
    for (int i = 2; i < argc; ++i) {
      const std::string_view arg(argv[i]);
      if (arg == "--no-gps")
        pipeline_cfg.enable_gps = false;
      else if (arg == "--no-lidar")
        pipeline_cfg.enable_lidar = false;
      else if (arg == "--no-radar")
        pipeline_cfg.enable_radar = false;
      else if (arg == "--no-nhc")
        pipeline_cfg.enable_nhc = false;
      else if (arg == "--no-fwdvel")
        pipeline_cfg.enable_fwdvel = false;
      else if (arg == "--gps-max" && i + 1 < argc) {
        pipeline_cfg.gps_max_measurements = std::atoi(argv[++i]);
      }
    }
  }

  // 1. Load
  std::cout << "Loading " << mcap_path.filename() << " ...\n";
  nufuse::domain::SceneData scene;
  try {
    scene = nufuse::io::loadMcap(mcap_path);
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return EXIT_FAILURE;
  }

  if (!validateScene(scene)) return EXIT_FAILURE;

  std::cout << "  IMU: " << scene.imu.size() << "  GNSS: " << scene.gnss.size()
            << "  Odom: " << scene.odom.size() << "  LiDAR: " << scene.lidar.size()
            << "  Radar: " << scene.radar.size() << "\n";

  // 2. LiDAR odometry
  std::unordered_map<uint64_t, gtsam::Pose3> lidar_rel;
  if (pipeline_cfg.enable_lidar) {
    std::cout << "\nComputing LiDAR GICP odometry...\n";
    auto lidar_poses = nufuse::processing::computeLidarOdometry(
        scene.lidar, *scene.extrinsics.body_from_lidar_top);
    std::cout << "  LiDAR pairs: " << lidar_poses.size() << "  converged: "
              << std::count_if(lidar_poses.begin(), lidar_poses.end(),
                               [](const auto& p) { return p.converged; })
              << "\n";
    lidar_rel = buildLidarRelMap(lidar_poses);
  }

  // 3. Merge measurements
  auto storage =
      nufuse::processing::mergeMeasurements(scene, lidar_rel, pipeline_cfg, init_from_gt);

  // 4. Radar processing
  if (pipeline_cfg.enable_radar && !scene.radar.empty()) {
    std::cout << "\nProcessing radar data (" << scene.radar.size() << " scans)...\n";
    auto radar_cfg = nufuse::processing::RadarProcessorConfig::fromPipelineConfig(pipeline_cfg);
    storage.radar_factors =
        nufuse::processing::processRadarScans(scene.radar, scene.extrinsics, scene.odom, radar_cfg);
    std::cout << "  Radar factors: " << storage.radar_factors.size() << "\n";
  }

  // 5. NHC + forward velocity
  if (pipeline_cfg.enable_nhc) {
    storage.nhc_factors = nufuse::processing::generateNhcFactors(storage, scene.odom);
    if (pipeline_cfg.enable_fwdvel) {
      storage.forward_velocity_factors =
          nufuse::processing::generateForwardVelocityFactors(storage, scene.odom);
    }
  }

  // 6. Build graph & optimize
  auto graph_bundle = nufuse::graph::buildGraph(storage, scene.extrinsics, pipeline_cfg);
  std::cout << "Graph: " << graph_bundle.graph.size() << " factors, " << graph_bundle.initial.size()
            << " variables, " << graph_bundle.num_keyframes + 1 << " keyframes, "
            << graph_bundle.num_corrupted << " corrupted GNSS fixes\n";

  auto results = nufuse::results::optimize(graph_bundle, storage);

  // 7. Report
  nufuse::results::printTrajectory(results);
  nufuse::results::printBias(results);
  nufuse::results::printError(results, scene, init_from_gt);
  nufuse::results::printLidarExtrinsics(results, scene);
  nufuse::results::printRadarExtrinsics(results, scene);

  if (pipeline_cfg.enable_fwdvel && !storage.forward_velocity_factors.empty()) {
    std::cout << "\nOdometry scale: " << (1.0 + results.odom_scale_delta.value)
              << " (delta=" << results.odom_scale_delta.value << ")\n";
  }

  // 8. Export merged MCAP
  if (output_path) {
    nufuse::io::writeResultsMcap(mcap_path, *output_path, results, storage, scene);
  }

  return EXIT_SUCCESS;
}
