/// @file domain/calibration.hpp
/// @brief Static sensor extrinsic calibration.

#pragma once

#include "core/types.hpp"

#include <optional>

namespace nufuse::domain {

/// @brief Static extrinsic calibration for all sensors (body <- sensor).
struct ExtrinsicCalibration {
    std::optional<core::BodyFromLidar> body_from_lidar_top;

    std::optional<core::BodyFromCamFront>      body_from_cam_front;
    std::optional<core::BodyFromCamFrontLeft>   body_from_cam_front_left;
    std::optional<core::BodyFromCamFrontRight>  body_from_cam_front_right;
    std::optional<core::BodyFromCamBack>        body_from_cam_back;
    std::optional<core::BodyFromCamBackLeft>    body_from_cam_back_left;
    std::optional<core::BodyFromCamBackRight>   body_from_cam_back_right;

    std::optional<core::BodyFromRadarFront>      body_from_radar_front;
    std::optional<core::BodyFromRadarFrontLeft>  body_from_radar_front_left;
    std::optional<core::BodyFromRadarFrontRight> body_from_radar_front_right;
    std::optional<core::BodyFromRadarBackLeft>   body_from_radar_back_left;
    std::optional<core::BodyFromRadarBackRight>  body_from_radar_back_right;
};

}  // namespace nufuse::domain
