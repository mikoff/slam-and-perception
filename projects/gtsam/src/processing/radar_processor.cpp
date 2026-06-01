/// @file processing/radar_processor.cpp
/// @brief Implementation of radar point cloud processing pipeline.

#include "processing/radar_processor.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <random>
#include <string_view>

#include "core/gtsam_boundary.hpp"
#include "processing/geo_utils.hpp"

namespace nufuse::processing {
namespace {

// ─── NuScenes dyn_prop classification values ─────────────────────────────────
// 0 = moving, 1 = stationary, 2 = stationary_candidate,
// 3 = moving_candidate, 4+ = other
constexpr int kDynPropStationary = 1;
constexpr int kDynPropStationaryCandidate = 2;

bool isStationary(int dyn_prop) {
  return dyn_prop == kDynPropStationary || dyn_prop == kDynPropStationaryCandidate;
}

// ─── Point cloud field parsing ───────────────────────────────────────────────

struct RadarFieldOffsets {
  uint32_t x = 0, y = 0, z = 0;
  uint32_t vx = UINT32_MAX, vy = UINT32_MAX;            // uncompensated velocity
  uint32_t vx_comp = UINT32_MAX, vy_comp = UINT32_MAX;  // compensated (for RANSAC filtering only)
  uint32_t dyn_prop = UINT32_MAX;
  bool valid = false;
};

RadarFieldOffsets findRadarFields(const foxglove::PointCloud& cloud) {
  RadarFieldOffsets offsets;
  for (int i = 0; i < cloud.fields_size(); ++i) {
    const auto& f = cloud.fields(i);
    if (f.name() == "x")
      offsets.x = f.offset();
    else if (f.name() == "y")
      offsets.y = f.offset();
    else if (f.name() == "z")
      offsets.z = f.offset();
    else if (f.name() == "vx")
      offsets.vx = f.offset();
    else if (f.name() == "vy")
      offsets.vy = f.offset();
    else if (f.name() == "vx_comp")
      offsets.vx_comp = f.offset();
    else if (f.name() == "vy_comp")
      offsets.vy_comp = f.offset();
    else if (f.name() == "dyn_prop")
      offsets.dyn_prop = f.offset();
  }
  // Need either uncompensated or compensated velocities
  offsets.valid = ((offsets.vx != UINT32_MAX && offsets.vy != UINT32_MAX) ||
                   (offsets.vx_comp != UINT32_MAX && offsets.vy_comp != UINT32_MAX)) &&
                  offsets.dyn_prop != UINT32_MAX;
  return offsets;
}

std::vector<RadarDetection> parseRadarCloud(const foxglove::PointCloud& cloud) {
  const auto offsets = findRadarFields(cloud);
  if (!offsets.valid) return {};

  const auto& data = cloud.data();
  const uint32_t stride = cloud.point_stride();
  if (stride == 0 || data.empty()) return {};

  const std::size_t num_points = data.size() / stride;
  std::vector<RadarDetection> detections;
  detections.reserve(num_points);

  // Determine which velocity fields to use for factor measurement
  const bool has_uncompensated = (offsets.vx != UINT32_MAX && offsets.vy != UINT32_MAX);
  const bool has_compensated = (offsets.vx_comp != UINT32_MAX && offsets.vy_comp != UINT32_MAX);

  const auto* raw = reinterpret_cast<const uint8_t*>(data.data());
  for (std::size_t i = 0; i < num_points; ++i) {
    const auto* ptr = raw + i * stride;
    RadarDetection det;
    std::memcpy(&det.x, ptr + offsets.x, sizeof(float));
    std::memcpy(&det.y, ptr + offsets.y, sizeof(float));
    std::memcpy(&det.z, ptr + offsets.z, sizeof(float));

    // Read uncompensated velocity (for factor)
    if (has_uncompensated) {
      std::memcpy(&det.vx, ptr + offsets.vx, sizeof(float));
      std::memcpy(&det.vy, ptr + offsets.vy, sizeof(float));
    } else {
      det.vx = 0.0f;
      det.vy = 0.0f;
    }

    // Read compensated velocity (for RANSAC filtering)
    if (has_compensated) {
      std::memcpy(&det.vx_comp, ptr + offsets.vx_comp, sizeof(float));
      std::memcpy(&det.vy_comp, ptr + offsets.vy_comp, sizeof(float));
    } else {
      det.vx_comp = det.vx;
      det.vy_comp = det.vy;
    }

    // dyn_prop is typically stored as uint8 or int8
    uint8_t dp_raw = 0;
    std::memcpy(&dp_raw, ptr + offsets.dyn_prop, sizeof(uint8_t));
    det.dyn_prop = static_cast<int>(dp_raw);

    if (std::isfinite(det.x) && std::isfinite(det.y) && std::isfinite(det.vx)) {
      detections.push_back(det);
    }
  }
  return detections;
}

// ─── dyn_prop filtering ──────────────────────────────────────────────────────

std::vector<RadarDetection> filterStationary(const std::vector<RadarDetection>& detections) {
  std::vector<RadarDetection> filtered;
  filtered.reserve(detections.size());
  std::copy_if(detections.begin(), detections.end(), std::back_inserter(filtered),
               [](const RadarDetection& d) { return isStationary(d.dyn_prop); });
  return filtered;
}

// ─── 2D RANSAC for velocity consensus ───────────────────────────────────────
//
// For stationary points, the measured compensated velocity should be ~zero in
// the world frame. We use a 2D velocity (vx_comp, vy_comp) RANSAC to find
// the consensus velocity vector and discard outliers.

struct RansacResult {
  Eigen::Vector2d consensus_velocity;
  std::vector<int> inlier_indices;
};

RansacResult ransac2dVelocity(const std::vector<RadarDetection>& detections,
                              const RadarProcessorConfig& config) {
  RansacResult best;
  if (detections.size() < 2) return best;

  std::mt19937 rng(42);
  std::uniform_int_distribution<int> dist(0, static_cast<int>(detections.size()) - 1);

  for (int iter = 0; iter < config.ransac_max_iterations; ++iter) {
    // Pick a random sample point as hypothesis
    const int idx = dist(rng);
    const Eigen::Vector2d hypothesis(detections[idx].vx_comp, detections[idx].vy_comp);

    // Count inliers
    std::vector<int> inliers;
    for (int i = 0; i < static_cast<int>(detections.size()); ++i) {
      const Eigen::Vector2d v(detections[i].vx_comp, detections[i].vy_comp);
      if ((v - hypothesis).norm() < config.ransac_inlier_threshold) {
        inliers.push_back(i);
      }
    }

    if (static_cast<int>(inliers.size()) > static_cast<int>(best.inlier_indices.size())) {
      // Recompute consensus as mean of inliers
      Eigen::Vector2d mean = Eigen::Vector2d::Zero();
      for (int i : inliers) {
        mean += Eigen::Vector2d(detections[i].vx_comp, detections[i].vy_comp);
      }
      mean /= static_cast<double>(inliers.size());
      best.consensus_velocity = mean;
      best.inlier_indices = std::move(inliers);
    }
  }
  return best;
}

// ─── Radar extrinsic key assignment ──────────────────────────────────────────

struct RadarSensorInfo {
  gtsam::Key extrinsic_key;
  gtsam::Pose3 T_BR;
};

std::string extractRadarName(const foxglove::PointCloud& cloud) {
  // The frame_id in foxglove PointCloud indicates which radar sensor
  return cloud.frame_id();
}

std::optional<RadarSensorInfo> getRadarInfo(const std::string& frame_id,
                                            const domain::ExtrinsicCalibration& extrinsics) {
  // Use different symbol indices for each radar: R(0)..R(4)
  using gtsam::Symbol;

  if (frame_id == "RADAR_FRONT" && extrinsics.body_from_radar_front) {
    return RadarSensorInfo{Symbol('R', 0), extrinsics.body_from_radar_front->value()};
  } else if (frame_id == "RADAR_FRONT_LEFT" && extrinsics.body_from_radar_front_left) {
    return RadarSensorInfo{Symbol('R', 1), extrinsics.body_from_radar_front_left->value()};
  } else if (frame_id == "RADAR_FRONT_RIGHT" && extrinsics.body_from_radar_front_right) {
    return RadarSensorInfo{Symbol('R', 2), extrinsics.body_from_radar_front_right->value()};
  } else if (frame_id == "RADAR_BACK_LEFT" && extrinsics.body_from_radar_back_left) {
    return RadarSensorInfo{Symbol('R', 3), extrinsics.body_from_radar_back_left->value()};
  } else if (frame_id == "RADAR_BACK_RIGHT" && extrinsics.body_from_radar_back_right) {
    return RadarSensorInfo{Symbol('R', 4), extrinsics.body_from_radar_back_right->value()};
  }
  return std::nullopt;
}

// ─── Bearing vector with z-squash ────────────────────────────────────────────

gtsam::Vector3 makeSquashedBearing(float x, float y) {
  // Squash z to 0, project onto 2D plane, then normalize in 3D with z=0
  Eigen::Vector3d bearing(static_cast<double>(x), static_cast<double>(y), 0.0);
  const double norm = bearing.norm();
  if (norm < 1e-10) return gtsam::Vector3(1.0, 0.0, 0.0);  // fallback
  return bearing / norm;
}

// ─── Angular velocity lookup ─────────────────────────────────────────────────

gtsam::Vector3 getAngularVelocity(const std::vector<domain::OdomMeasurement>& odom,
                                  uint64_t stamp_ns) {
  if (odom.empty()) return gtsam::Vector3::Zero();
  const auto interp = interpolateOdom(odom, stamp_ns);
  return interp.angular_velocity.value();
}

}  // anonymous namespace

// ─── Public interface ────────────────────────────────────────────────────────

std::vector<graph::StoredRadarFactor> processRadarScans(
    const std::vector<core::Stamped<foxglove::PointCloud>>& radar_scans,
    const domain::ExtrinsicCalibration& extrinsics,
    const std::vector<domain::OdomMeasurement>& odom, const RadarProcessorConfig& config) {
  std::vector<graph::StoredRadarFactor> factors;
  factors.reserve(radar_scans.size());

  for (const auto& scan : radar_scans) {
    const std::string frame_id = extractRadarName(scan.data);
    const auto sensor_info = getRadarInfo(frame_id, extrinsics);
    if (!sensor_info) continue;

    // 1. Parse point cloud
    auto detections = parseRadarCloud(scan.data);
    if (detections.empty()) continue;

    // 2. Filter by dyn_prop: keep only stationary / stationary_candidate
    auto stationary = filterStationary(detections);
    if (static_cast<int>(stationary.size()) < config.ransac_min_inliers) continue;

    // 3. 2D RANSAC for velocity consensus
    auto ransac = ransac2dVelocity(stationary, config);
    if (static_cast<int>(ransac.inlier_indices.size()) < config.ransac_min_inliers) continue;

    // 4. Get angular velocity at this timestamp
    const core::AngularVelocityBody omega_B(getAngularVelocity(odom, scan.stamp.value()));

    // 5. Build measurements from inliers (capped to avoid dominating the graph)
    graph::StoredRadarFactor factor;
    factor.stamp = scan.stamp;
    factor.extrinsic_key = sensor_info->extrinsic_key;

    // Pick up to max_measurements_per_scan inliers, preferring those closest to consensus
    auto& inliers = ransac.inlier_indices;
    if (static_cast<int>(inliers.size()) > config.max_measurements_per_scan) {
      // Sort by distance to consensus velocity, keep closest
      std::sort(inliers.begin(), inliers.end(), [&](int a, int b) {
        const Eigen::Vector2d va(stationary[a].vx_comp, stationary[a].vy_comp);
        const Eigen::Vector2d vb(stationary[b].vx_comp, stationary[b].vy_comp);
        return (va - ransac.consensus_velocity).squaredNorm() <
               (vb - ransac.consensus_velocity).squaredNorm();
      });
      inliers.resize(config.max_measurements_per_scan);
    }

    for (int idx : inliers) {
      const auto& det = stationary[idx];

      // Use UNCOMPENSATED velocity (vx, vy) for factor measurement.
      // For stationary targets: vx,vy ≈ -v_ego_in_radar, which matches the factor model:
      //   predicted = -u_R^T * v_ego_in_radar
      const gtsam::Vector3 bearing_raw = makeSquashedBearing(det.x, det.y);
      const Eigen::Vector2d vel_2d(det.vx, det.vy);
      const double v_radial = vel_2d.dot(Eigen::Vector2d(bearing_raw.x(), bearing_raw.y()));

      graph::StoredRadarMeasurement meas;
      meas.v_radial = v_radial;
      meas.bearing = core::RadarBearing(bearing_raw);
      meas.omega_B = omega_B;
      factor.measurements.push_back(meas);
    }

    if (!factor.measurements.empty()) {
      factors.push_back(std::move(factor));
    }
  }

  return factors;
}

}  // namespace nufuse::processing
