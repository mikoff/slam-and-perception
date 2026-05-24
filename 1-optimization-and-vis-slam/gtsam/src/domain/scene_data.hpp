/// @file domain/scene_data.hpp
/// @brief Type-safe container for all sensor data from a single scene.

#pragma once

#include "domain/calibration.hpp"
#include "domain/measurements.hpp"

#include "foxglove/CompressedImage.pb.h"
#include "foxglove/PointCloud.pb.h"

#include <vector>

namespace nufuse::domain {

/// @brief All sensor data extracted from a single scene file.
struct SceneData {
    std::vector<ImuMeasurement> imu;
    std::vector<OdomMeasurement> odom;
    std::vector<GnssFix> gnss;
    std::vector<core::Stamped<foxglove::PointCloud>> lidar;
    std::vector<core::Stamped<foxglove::PointCloud>> radar;
    std::vector<core::Stamped<foxglove::CompressedImage>> images;

    ExtrinsicCalibration extrinsics;

    std::vector<core::Stamped<core::MapFromBody>> ego_poses;
};

}  // namespace nufuse::domain
