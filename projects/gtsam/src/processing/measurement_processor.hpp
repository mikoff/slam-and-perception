/// @file processing/measurement_processor.hpp
/// @brief Incremental measurement processor producing factor storage.

#pragma once

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <optional>
#include <vector>

#include "core/pipeline_config.hpp"
#include "domain/measurements.hpp"
#include "graph/factor_storage.hpp"

namespace nufuse::processing {

/// @brief Configuration for measurement processing.
struct ProcessorConfig {
  bool init_from_gt = false;                  // Use ground truth position for first keyframe
  gtsam::imuBias::ConstantBias initial_bias;  // Pre-computed initial IMU bias
  core::ImuIntrinsicsConfig imu;              // IMU noise parameters
  double keyframe_interval_s = 0.5;           // IMU-only keyframe interval
};

/// @brief Processes raw sensor measurements into a FactorStorage.
///
/// Feed measurements via addImu/addGnss/addLidar in chronological order.
/// Call finalize() to retrieve the completed storage.
class MeasurementProcessor {
 public:
  MeasurementProcessor(const ProcessorConfig& cfg,
                       const std::vector<domain::OdomMeasurement>& odom);

  void addImu(const domain::ImuMeasurement& m);

  /// @brief Add a GNSS fix. The corrupted flag is set externally (by GpsCorruptor).
  void addGnss(const domain::GnssFix& fix, bool corrupted = false);

  void addLidar(core::Timestamp stamp, std::optional<gtsam::Pose3> rel_pose);

  graph::FactorStorage finalize() &&;

 private:
  void createKeyframe(uint64_t stamp_ns);
  void initializeFirstKeyframe(uint64_t stamp_ns);
  void storeImuFactor(uint64_t stamp_ns);
  void storeKeyframeEstimate(uint64_t stamp_ns);

  const ProcessorConfig& cfg_;
  const std::vector<domain::OdomMeasurement>& odom_;
  gtsam::imuBias::ConstantBias bias0_;
  gtsam::PreintegratedCombinedMeasurements pim_;
  graph::FactorStorage storage_;

  std::optional<domain::ImuMeasurement> prev_imu_;
  double lat0_ = 0.0, lon0_ = 0.0, alt0_ = 0.0;
  gtsam::Point3 odom_origin_;
  int idx_ = -1;
  int lidar_prev_idx_ = -1;
  uint64_t last_imu_keyframe_stamp_ns_ = 0;
  uint64_t keyframe_interval_ns_;
};

}  // namespace nufuse::processing
