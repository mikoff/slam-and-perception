/// @file nuscenes_mcap_loader.hpp
/// @brief Reads NuScenes MCAP files and returns typed sensor data vectors.
///
/// Raw layer:   protobuf messages (PointCloud, CompressedImage)
/// Domain layer: frame-tagged types (ImuMeasurement, OdomMeasurement, GnssFix,
///               ExtrinsicCalibration, ego poses)

#pragma once

#include "nuscenes_domain_types.hpp"

#include "foxglove/CompressedImage.pb.h"
#include "foxglove/PointCloud.pb.h"

#include <filesystem>
#include <vector>

namespace nuscenes {

/// @brief All sensor data extracted from a single MCAP scene file.
struct SceneData {
    std::vector<ImuMeasurement> imu;
    std::vector<OdomMeasurement> odom;
    std::vector<GnssFix> gnss;
    std::vector<Stamped<foxglove::PointCloud>> lidar;
    std::vector<Stamped<foxglove::PointCloud>> radar;
    std::vector<Stamped<foxglove::CompressedImage>> images;

    /// @brief Static sensor extrinsics (body → sensor), extracted from /tf.
    ExtrinsicCalibration extrinsics;

    /// @brief Dynamic ego poses (map → body), extracted from /tf.
    std::vector<Stamped<slam::geometry::Transform<gtsam::Pose3, MapFrame, BodyFrame>>>
        ego_poses;
};

/// @brief Loads all supported sensor data from a NuScenes MCAP file.
///
/// @param mcap_path Path to the .mcap file.
/// @return SceneData containing all decoded messages.
/// @throws std::runtime_error if the file cannot be opened.
SceneData loadMcap(const std::filesystem::path& mcap_path);

}  // namespace nuscenes
