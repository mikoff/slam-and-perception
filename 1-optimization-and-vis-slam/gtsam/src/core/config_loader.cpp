/// @file core/config_loader.cpp
/// @brief Implementation of JSON config loading.

#include "core/config_loader.hpp"

#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>

namespace nufuse::core {

using json = nlohmann::json;

namespace {

template <typename T>
void readIfPresent(const json& j, const char* key, T& out) {
  if (j.contains(key)) {
    out = j[key].get<T>();
  }
}

}  // namespace

PipelineConfig loadConfig(const std::filesystem::path& json_path) {
  std::ifstream file(json_path);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open config file: " + json_path.string());
  }

  const json root = json::parse(file);
  PipelineConfig cfg;

  // Sensor enable flags
  if (root.contains("sensors")) {
    const auto& s = root["sensors"];
    readIfPresent(s, "enable_gps", cfg.enable_gps);
    readIfPresent(s, "enable_lidar", cfg.enable_lidar);
    readIfPresent(s, "enable_radar", cfg.enable_radar);
    readIfPresent(s, "enable_nhc", cfg.enable_nhc);
    readIfPresent(s, "enable_fwdvel", cfg.enable_fwdvel);
    readIfPresent(s, "gps_max_measurements", cfg.gps_max_measurements);
  }

  // Prior noise
  if (root.contains("prior")) {
    const auto& p = root["prior"];
    readIfPresent(p, "pose_rot_sigma", cfg.prior.pose_rot_sigma);
    readIfPresent(p, "pose_trans_sigma", cfg.prior.pose_trans_sigma);
    readIfPresent(p, "velocity_sigma", cfg.prior.velocity_sigma);
    readIfPresent(p, "bias_accel_sigma", cfg.prior.bias_accel_sigma);
    readIfPresent(p, "bias_gyro_sigma", cfg.prior.bias_gyro_sigma);
    readIfPresent(p, "bias_gyro_sigma_obs", cfg.prior.bias_gyro_sigma_obs);
  }

  // GPS noise
  if (root.contains("gps")) {
    const auto& g = root["gps"];
    readIfPresent(g, "sigma", cfg.gps.sigma);
    readIfPresent(g, "robust_threshold", cfg.gps.robust_threshold);
  }

  // LiDAR noise
  if (root.contains("lidar")) {
    const auto& l = root["lidar"];
    readIfPresent(l, "odom_rot_sigma", cfg.lidar.odom_rot_sigma);
    readIfPresent(l, "odom_trans_sigma", cfg.lidar.odom_trans_sigma);
    readIfPresent(l, "robust_threshold", cfg.lidar.robust_threshold);
    readIfPresent(l, "extrinsic_prior_rot_sigma", cfg.lidar.extrinsic_prior_rot_sigma);
    readIfPresent(l, "extrinsic_prior_trans_sigma", cfg.lidar.extrinsic_prior_trans_sigma);
    readIfPresent(l, "min_inliers", cfg.lidar.min_inliers);
    readIfPresent(l, "max_registration_error", cfg.lidar.max_registration_error);
    readIfPresent(l, "max_translation_m", cfg.lidar.max_translation_m);
    readIfPresent(l, "imu_consistency_threshold", cfg.lidar.imu_consistency_threshold);
  }

  // Radar noise
  if (root.contains("radar")) {
    const auto& r = root["radar"];
    readIfPresent(r, "velocity_sigma", cfg.radar.velocity_sigma);
    readIfPresent(r, "robust_threshold", cfg.radar.robust_threshold);
    readIfPresent(r, "extrinsic_prior_rot_sigma", cfg.radar.extrinsic_prior_rot_sigma);
    readIfPresent(r, "extrinsic_prior_yaw_sigma", cfg.radar.extrinsic_prior_yaw_sigma);
    readIfPresent(r, "extrinsic_prior_trans_xy", cfg.radar.extrinsic_prior_trans_xy);
    readIfPresent(r, "extrinsic_prior_trans_z", cfg.radar.extrinsic_prior_trans_z);
  }

  // NHC
  if (root.contains("nhc")) {
    const auto& n = root["nhc"];
    readIfPresent(n, "lateral_sigma", cfg.nhc.lateral_sigma);
    readIfPresent(n, "vertical_sigma", cfg.nhc.vertical_sigma);
    readIfPresent(n, "lever_arm_x", cfg.nhc.lever_arm_x);
    readIfPresent(n, "lever_arm_y", cfg.nhc.lever_arm_y);
    readIfPresent(n, "lever_arm_z", cfg.nhc.lever_arm_z);
  }

  // Forward velocity
  if (root.contains("forward_velocity")) {
    const auto& f = root["forward_velocity"];
    readIfPresent(f, "sigma", cfg.forward_velocity.sigma);
    readIfPresent(f, "scale_prior_sigma", cfg.forward_velocity.scale_prior_sigma);
  }

  // Radar processing (RANSAC)
  if (root.contains("radar_processing")) {
    const auto& rp = root["radar_processing"];
    readIfPresent(rp, "ransac_max_iterations", cfg.radar_processing.ransac_max_iterations);
    readIfPresent(rp, "ransac_inlier_threshold", cfg.radar_processing.ransac_inlier_threshold);
    readIfPresent(rp, "ransac_min_inliers", cfg.radar_processing.ransac_min_inliers);
    readIfPresent(rp, "max_measurements_per_scan", cfg.radar_processing.max_measurements_per_scan);
  }

  // IMU intrinsics
  if (root.contains("imu")) {
    const auto& i = root["imu"];
    readIfPresent(i, "accel_noise_density", cfg.imu.accel_noise_density);
    readIfPresent(i, "gyro_noise_density", cfg.imu.gyro_noise_density);
    readIfPresent(i, "integration_sigma", cfg.imu.integration_sigma);
    readIfPresent(i, "bias_accel_sigma", cfg.imu.bias_accel_sigma);
    readIfPresent(i, "bias_gyro_sigma", cfg.imu.bias_gyro_sigma);
    readIfPresent(i, "gravity", cfg.imu.gravity);
  }

  // GPS corruption (testing)
  if (root.contains("gps_corruption")) {
    const auto& gc = root["gps_corruption"];
    readIfPresent(gc, "enable", cfg.gps_corruption.enable);
    readIfPresent(gc, "spike_metres", cfg.gps_corruption.spike_metres);
    readIfPresent(gc, "spike_fraction", cfg.gps_corruption.spike_fraction);
    readIfPresent(gc, "rng_seed", cfg.gps_corruption.rng_seed);
  }

  // Keyframe interval
  readIfPresent(root, "imu_keyframe_interval_s", cfg.imu_keyframe_interval_s);

  // Robust kernel type
  if (root.contains("robust_kernel")) {
    const auto& val = root["robust_kernel"].get<std::string>();
    if (val == "cauchy")
      cfg.robust_kernel = RobustKernelType::Cauchy;
    else if (val == "huber")
      cfg.robust_kernel = RobustKernelType::Huber;
  }

  // Optimizer type
  if (root.contains("optimizer")) {
    const auto& val = root["optimizer"].get<std::string>();
    if (val == "lm")
      cfg.optimizer = OptimizerType::LM;
    else if (val == "gnc_lm")
      cfg.optimizer = OptimizerType::GNC_LM;
  }

  // GNC parameters
  if (root.contains("gnc")) {
    const auto& g = root["gnc"];
    readIfPresent(g, "mu_step", cfg.gnc.mu_step);
    readIfPresent(g, "relative_cost_tol", cfg.gnc.relative_cost_tol);
    readIfPresent(g, "weights_tol", cfg.gnc.weights_tol);
  }

  return cfg;
}

}  // namespace nufuse::core
