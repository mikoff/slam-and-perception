/// @file processing/lidar_odometry.cpp
/// @brief Implementation of LiDAR scan-to-scan GICP registration.

#include "processing/lidar_odometry.hpp"

#include <small_gicp/registration/registration_helper.hpp>

#include <cstring>
#include <vector>

namespace nufuse::processing {
namespace {

std::vector<Eigen::Vector4f> toEigenPoints(const foxglove::PointCloud& cloud) {
    const auto& data = cloud.data();
    const uint32_t stride = cloud.point_stride();
    if (stride == 0 || data.empty()) return {};

    uint32_t off_x = 0, off_y = 4, off_z = 8;
    for (int i = 0; i < cloud.fields_size(); ++i) {
        const auto& f = cloud.fields(i);
        if (f.name() == "x")      off_x = f.offset();
        else if (f.name() == "y") off_y = f.offset();
        else if (f.name() == "z") off_z = f.offset();
    }

    const std::size_t num_points = data.size() / stride;
    std::vector<Eigen::Vector4f> points;
    points.reserve(num_points);

    const auto* raw = reinterpret_cast<const uint8_t*>(data.data());
    for (std::size_t i = 0; i < num_points; ++i) {
        const auto* ptr = raw + i * stride;
        float x, y, z;
        std::memcpy(&x, ptr + off_x, sizeof(float));
        std::memcpy(&y, ptr + off_y, sizeof(float));
        std::memcpy(&z, ptr + off_z, sizeof(float));
        if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)
            && (x != 0.0f || y != 0.0f || z != 0.0f)) {
            points.emplace_back(x, y, z, 1.0f);
        }
    }
    return points;
}

gtsam::Pose3 isometryToPose3(const Eigen::Isometry3d& T) {
    return gtsam::Pose3(gtsam::Rot3(T.rotation()), gtsam::Point3(T.translation()));
}

small_gicp::RegistrationSetting makeGicpSettings() {
    small_gicp::RegistrationSetting s;
    s.num_threads = 4;
    s.downsampling_resolution = 0.5;
    s.max_correspondence_distance = 2.0;
    return s;
}

}  // anonymous namespace

std::vector<LidarRelativePose> computeLidarOdometry(
    const std::vector<core::Stamped<foxglove::PointCloud>>& lidar_scans,
    const core::BodyFromLidar& body_from_lidar) {

    std::vector<LidarRelativePose> results;
    if (lidar_scans.size() < 2) return results;

    const auto lidar_from_body = body_from_lidar.inverse();
    const auto setting = makeGicpSettings();

    auto prev_points = toEigenPoints(lidar_scans[0].data);
    auto [prev_cloud, prev_tree] = small_gicp::preprocess_points(
        prev_points, setting.downsampling_resolution, 10, setting.num_threads);

    for (std::size_t i = 1; i < lidar_scans.size(); ++i) {
        auto curr_points = toEigenPoints(lidar_scans[i].data);
        auto [curr_cloud, curr_tree] = small_gicp::preprocess_points(
            curr_points, setting.downsampling_resolution, 10, setting.num_threads);

        auto reg = small_gicp::align(
            *prev_cloud, *curr_cloud, *prev_tree,
            Eigen::Isometry3d::Identity(), setting);

        core::LidarDelta T_lidar(isometryToPose3(reg.T_target_source));
        core::BodyDelta T_body = body_from_lidar * T_lidar * lidar_from_body;

        results.push_back({
            .stamp_from  = lidar_scans[i - 1].stamp,
            .stamp_to    = lidar_scans[i].stamp,
            .T_lidar     = T_lidar,
            .T_body      = T_body,
            .converged   = reg.converged,
            .error       = reg.error,
            .num_inliers = static_cast<int>(reg.num_inliers),
        });

        prev_cloud = curr_cloud;
        prev_tree  = curr_tree;
    }
    return results;
}

}  // namespace nufuse::processing
