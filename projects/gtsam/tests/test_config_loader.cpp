/// @file tests/test_config_loader.cpp
/// @brief Unit tests for JSON configuration loading.

#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>

#include "core/config_loader.hpp"

using namespace nufuse;

namespace {

/// Create a temporary JSON file and return its path.
std::filesystem::path writeTmpJson(const std::string& content) {
  auto path = std::filesystem::temp_directory_path() / "nufuse_test_config.json";
  std::ofstream f(path);
  f << content;
  return path;
}

}  // namespace

TEST(ConfigLoader, EmptyJsonReturnsDefaults) {
  auto path = writeTmpJson("{}");
  auto cfg = core::loadConfig(path);
  // Defaults from PipelineConfig
  EXPECT_TRUE(cfg.enable_gps);
  EXPECT_TRUE(cfg.enable_lidar);
  EXPECT_TRUE(cfg.enable_radar);
  EXPECT_TRUE(cfg.enable_nhc);
  EXPECT_TRUE(cfg.enable_fwdvel);
  std::filesystem::remove(path);
}

TEST(ConfigLoader, SensorsOverride) {
  auto path = writeTmpJson(R"({
    "sensors": {
      "enable_gps": false,
      "enable_radar": false,
      "gps_max_measurements": 42
    }
  })");
  auto cfg = core::loadConfig(path);
  EXPECT_FALSE(cfg.enable_gps);
  EXPECT_TRUE(cfg.enable_lidar);  // not overridden
  EXPECT_FALSE(cfg.enable_radar);
  EXPECT_EQ(cfg.gps_max_measurements, 42);
  std::filesystem::remove(path);
}

TEST(ConfigLoader, PriorNoise) {
  auto path = writeTmpJson(R"({
    "prior": {
      "pose_rot_sigma": 0.5,
      "velocity_sigma": 0.1
    }
  })");
  auto cfg = core::loadConfig(path);
  EXPECT_DOUBLE_EQ(cfg.prior.pose_rot_sigma, 0.5);
  EXPECT_DOUBLE_EQ(cfg.prior.velocity_sigma, 0.1);
  // Default remains for untouched fields
  EXPECT_DOUBLE_EQ(cfg.prior.pose_trans_sigma, core::PipelineConfig{}.prior.pose_trans_sigma);
  std::filesystem::remove(path);
}

TEST(ConfigLoader, GpsCorruption) {
  auto path = writeTmpJson(R"({
    "gps_corruption": {
      "enable": true,
      "spike_metres": 99.0,
      "spike_fraction": 0.3,
      "rng_seed": 123
    }
  })");
  auto cfg = core::loadConfig(path);
  EXPECT_TRUE(cfg.gps_corruption.enable);
  EXPECT_DOUBLE_EQ(cfg.gps_corruption.spike_metres, 99.0);
  EXPECT_DOUBLE_EQ(cfg.gps_corruption.spike_fraction, 0.3);
  EXPECT_EQ(cfg.gps_corruption.rng_seed, 123);
  std::filesystem::remove(path);
}

TEST(ConfigLoader, ImuParams) {
  auto path = writeTmpJson(R"({
    "imu": {
      "gravity": 9.80,
      "accel_noise_density": 0.005
    }
  })");
  auto cfg = core::loadConfig(path);
  EXPECT_DOUBLE_EQ(cfg.imu.gravity, 9.80);
  EXPECT_DOUBLE_EQ(cfg.imu.accel_noise_density, 0.005);
  std::filesystem::remove(path);
}

TEST(ConfigLoader, MissingFileThrows) {
  EXPECT_THROW(core::loadConfig("/nonexistent/path.json"), std::runtime_error);
}

TEST(ConfigLoader, KeyframeInterval) {
  auto path = writeTmpJson(R"({
    "imu_keyframe_interval_s": 0.25
  })");
  auto cfg = core::loadConfig(path);
  EXPECT_DOUBLE_EQ(cfg.imu_keyframe_interval_s, 0.25);
  std::filesystem::remove(path);
}
