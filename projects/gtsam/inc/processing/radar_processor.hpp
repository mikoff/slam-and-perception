/// @file processing/radar_processor.hpp
/// @brief Radar point cloud processing: dyn_prop filtering, 2D RANSAC velocity consensus,
///        and conversion to StoredRadarFactor measurements.

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <string>
#include <vector>

#include "core/pipeline_config.hpp"
#include "core/types.hpp"
#include "domain/calibration.hpp"
#include "domain/measurements.hpp"
#include "foxglove/PointCloud.pb.h"
#include "graph/factor_types.hpp"

namespace nufuse::processing {

/// @brief Configuration for radar processing.
struct RadarProcessorConfig {
  int ransac_max_iterations = 100;
  double ransac_inlier_threshold = 0.3;
  int ransac_min_inliers = 3;
  int max_measurements_per_scan = 5;

  /// @brief Construct from PipelineConfig radar_processing section.
  static RadarProcessorConfig fromPipelineConfig(const core::PipelineConfig& cfg) {
    return {cfg.radar_processing.ransac_max_iterations,
            cfg.radar_processing.ransac_inlier_threshold, cfg.radar_processing.ransac_min_inliers,
            cfg.radar_processing.max_measurements_per_scan};
  }
};

/// @brief A single parsed radar detection (before filtering).
struct RadarDetection {
  float x, y, z;           // position in radar frame
  float vx, vy;            // uncompensated velocity (raw, for factor measurement)
  float vx_comp, vy_comp;  // compensated velocity (for RANSAC/dyn_prop filtering)
  int dyn_prop;            // dynamic property classification
};

/// @brief Processes all radar point clouds into StoredRadarFactor entries.
///
/// Unified pipeline for all 5 radar sensors. Handles:
/// 1. Parsing the PointCloud protobuf into RadarDetection structs
/// 2. Filtering by dyn_prop (stationary or stationary_candidate)
/// 3. 2D RANSAC to find consensus velocity and reject outliers
/// 4. Z-squashing of bearing vectors for the factor
///
/// @param radar_scans     All radar point clouds with timestamps (from all radars)
/// @param extrinsics      Calibration containing radar extrinsics
/// @param odom            Odometry for angular velocity interpolation
/// @param config          Processing configuration
/// @return Vector of StoredRadarFactor, one per keyframe-aligned radar scan
std::vector<graph::StoredRadarFactor> processRadarScans(
    const std::vector<core::Stamped<foxglove::PointCloud>>& radar_scans,
    const domain::ExtrinsicCalibration& extrinsics,
    const std::vector<domain::OdomMeasurement>& odom, const RadarProcessorConfig& config = {});

}  // namespace nufuse::processing
