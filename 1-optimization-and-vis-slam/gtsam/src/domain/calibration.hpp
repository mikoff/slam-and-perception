/// @file domain/calibration.hpp
/// @brief Static sensor extrinsic calibration.

#pragma once

#include <optional>
#include <unordered_map>

#include "core/types.hpp"

namespace nufuse::domain {

/// @brief Identifies a sensor for extrinsic lookup.
enum class SensorId : uint8_t {
  LidarTop = 0,
  CamFront,
  CamFrontLeft,
  CamFrontRight,
  CamBack,
  CamBackLeft,
  CamBackRight,
  RadarFront,
  RadarFrontLeft,
  RadarFrontRight,
  RadarBackLeft,
  RadarBackRight,
};

/// @brief Static extrinsic calibration for all sensors (body <- sensor).
struct ExtrinsicCalibration {
  std::optional<core::BodyFromLidar> body_from_lidar_top;

  std::optional<core::BodyFromCamFront> body_from_cam_front;
  std::optional<core::BodyFromCamFrontLeft> body_from_cam_front_left;
  std::optional<core::BodyFromCamFrontRight> body_from_cam_front_right;
  std::optional<core::BodyFromCamBack> body_from_cam_back;
  std::optional<core::BodyFromCamBackLeft> body_from_cam_back_left;
  std::optional<core::BodyFromCamBackRight> body_from_cam_back_right;

  std::optional<core::BodyFromRadarFront> body_from_radar_front;
  std::optional<core::BodyFromRadarFrontLeft> body_from_radar_front_left;
  std::optional<core::BodyFromRadarFrontRight> body_from_radar_front_right;
  std::optional<core::BodyFromRadarBackLeft> body_from_radar_back_left;
  std::optional<core::BodyFromRadarBackRight> body_from_radar_back_right;

  /// @brief Lookup radar extrinsic by index (0=Front, 1=FrontLeft, 2=FrontRight,
  ///        3=BackLeft, 4=BackRight). Returns identity if not calibrated.
  gtsam::Pose3 radarExtrinsic(int idx) const {
    switch (idx) {
      case 0:
        return body_from_radar_front ? body_from_radar_front->value() : gtsam::Pose3::Identity();
      case 1:
        return body_from_radar_front_left ? body_from_radar_front_left->value()
                                          : gtsam::Pose3::Identity();
      case 2:
        return body_from_radar_front_right ? body_from_radar_front_right->value()
                                           : gtsam::Pose3::Identity();
      case 3:
        return body_from_radar_back_left ? body_from_radar_back_left->value()
                                         : gtsam::Pose3::Identity();
      case 4:
        return body_from_radar_back_right ? body_from_radar_back_right->value()
                                          : gtsam::Pose3::Identity();
      default:
        return gtsam::Pose3::Identity();
    }
  }
};

}  // namespace nufuse::domain
