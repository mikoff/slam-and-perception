/// @file main_nufuse.cpp
/// @brief CLI entry point for the NuFuse multi-sensor SLAM pipeline.

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <optional>

#include "core/config_loader.hpp"
#include "core/pipeline_config.hpp"
#include "domain/scene_data.hpp"
#include "io/mcap_loader.hpp"
#include "io/mcap_writer.hpp"
#include "pipeline/pipeline.hpp"
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

  // Load
  std::cout << "Running NuFuse on " << mcap_path.filename() << " ...\n";
  nufuse::domain::SceneData scene;
  try {
    scene = nufuse::io::loadMcap(mcap_path);
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return EXIT_FAILURE;
  }

  if (!validateScene(scene)) return EXIT_FAILURE;

  // Run pipeline
  auto [results, storage] = nufuse::pipeline::run(scene, pipeline_cfg, init_from_gt);

  // Report
  nufuse::results::printTrajectory(results);
  nufuse::results::printBias(results);
  nufuse::results::printError(results, scene, init_from_gt);
  nufuse::results::printLidarExtrinsics(results, scene);
  nufuse::results::printRadarExtrinsics(results, scene);

  if (pipeline_cfg.enable_fwdvel) {
    std::cout << "\nOdometry scale: " << (1.0 + results.odom_scale_delta.value)
              << " (delta=" << results.odom_scale_delta.value << ")\n";
  }

  // Export merged MCAP
  if (output_path) {
    nufuse::io::writeResultsMcap(mcap_path, *output_path, results, storage, scene);
  }

  std::cout << "Done.\n";
  return EXIT_SUCCESS;
}
