/// @file core/pipeline_config.hpp
/// @brief Complete pipeline configuration — sensor enables + all tunable noise parameters.
///
/// All parameters have compiled defaults matching the original constants.hpp values.
/// Can be overridden at runtime via JSON config file (see config_loader.hpp).

#pragma once

#include <cstdint>
#include <string>

namespace nufuse::core {

/// @brief Choice of robust kernel for measurement noise models.
enum class RobustKernelType { Huber, Cauchy };

/// @brief Optimizer backend selection.
enum class OptimizerType { LM, GNC_LM };

/// @brief Prior noise sigmas for the factor graph.
struct PriorNoiseConfig {
  double pose_rot_sigma = 0.05;      // rad
  double pose_trans_sigma = 0.5;     // m
  double velocity_sigma = 0.1;       // m/s (isotropic 3D)
  double bias_accel_sigma = 0.1;     // m/s²
  double bias_gyro_sigma = 0.003;    // rad/s (tight, dead-reckoning default)
  double bias_gyro_sigma_obs = 0.1;  // rad/s (loose, when heading is observable)
};

/// @brief GPS factor noise configuration.
struct GpsNoiseConfig {
  double sigma = 1.0;  // m (isotropic 3D)
  double robust_threshold = 1.345;
};

/// @brief LiDAR odometry and extrinsic noise configuration.
struct LidarNoiseConfig {
  double odom_rot_sigma = 0.05;               // rad
  double odom_trans_sigma = 0.1;              // m
  double robust_threshold = 0.5;              // Cauchy/Huber kernel parameter
  double extrinsic_prior_rot_sigma = 0.01;    // rad
  double extrinsic_prior_trans_sigma = 0.05;  // m

  // Quality gating — reject GICP results that fail these criteria
  // Set min_inliers=0 and imu_consistency_threshold=0 to disable gating entirely.
  int min_inliers = 0;                     // 0 = disabled (accept all converged registrations)
  double max_registration_error = 1e9;     // effectively disabled
  double max_translation_m = 1e9;          // effectively disabled
  double imu_consistency_threshold = 0.0;  // 0 = disabled (skip IMU cross-check)
};

/// @brief Radar velocity factor and extrinsic noise configuration.
struct RadarNoiseConfig {
  double velocity_sigma = 1.5;  // m/s
  double robust_threshold = 0.5;
  double extrinsic_prior_rot_sigma = 1e-4;  // rad (roll/pitch)
  double extrinsic_prior_yaw_sigma = 1e-2;  // rad
  double extrinsic_prior_trans_xy = 1e-2;   // m
  double extrinsic_prior_trans_z = 1e-4;    // m
};

/// @brief Non-holonomic constraint configuration.
struct NhcConfig {
  double lateral_sigma = 0.1;   // m/s
  double vertical_sigma = 0.1;  // m/s
  double lever_arm_x = 0.0;     // m (IMU to rear axle, body frame)
  double lever_arm_y = 0.0;
  double lever_arm_z = 0.0;
};

/// @brief Forward velocity scale factor configuration.
struct ForwardVelocityConfig {
  double sigma = 0.5;                // m/s
  double scale_prior_sigma = 0.001;  // near-locked for short runs
};

/// @brief Radar processing (RANSAC + filtering) configuration.
struct RadarProcessingConfig {
  int ransac_max_iterations = 100;
  double ransac_inlier_threshold = 0.3;  // m/s
  int ransac_min_inliers = 3;
  int max_measurements_per_scan = 5;
};

/// @brief IMU intrinsic noise parameters (sensor-specific).
struct ImuIntrinsicsConfig {
  double accel_noise_density = 0.0003924;   // m/s²/√Hz
  double gyro_noise_density = 0.000205689;  // rad/s/√Hz
  double integration_sigma = 1e-4;
  double bias_accel_sigma = 0.004905;  // m/s²
  double bias_gyro_sigma = 1.4544e-6;  // rad/s
  double gravity = 9.81;               // m/s² (positive-up for ENU)
};

/// @brief GPS corruption injection parameters (for testing).
struct GpsCorruptionConfig {
  bool enable = true;
  double spike_metres = 100.0;
  double spike_fraction = 0.20;
  unsigned rng_seed = 42;
};

/// @brief Complete pipeline configuration.
struct PipelineConfig {
  // Sensor enable/disable flags
  bool enable_gps = true;
  bool enable_lidar = true;
  bool enable_radar = true;
  bool enable_nhc = true;
  bool enable_fwdvel = true;
  int gps_max_measurements = 0;  // 0 = never disable

  // Noise model parameters (grouped by modality)
  PriorNoiseConfig prior;
  GpsNoiseConfig gps;
  LidarNoiseConfig lidar;
  RadarNoiseConfig radar;
  NhcConfig nhc;
  ForwardVelocityConfig forward_velocity;
  RadarProcessingConfig radar_processing;
  ImuIntrinsicsConfig imu;
  GpsCorruptionConfig gps_corruption;

  // Keyframe creation
  double imu_keyframe_interval_s = 0.5;

  // Robust kernel selection (applies to GPS, LiDAR, Radar)
  RobustKernelType robust_kernel = RobustKernelType::Cauchy;

  // Optimizer backend
  OptimizerType optimizer = OptimizerType::LM;

  // GNC optimizer parameters
  struct GncConfig {
    double mu_step = 1.4;
    double relative_cost_tol = 1e-5;
    double weights_tol = 1e-4;
  } gnc;
};

}  // namespace nufuse::core
