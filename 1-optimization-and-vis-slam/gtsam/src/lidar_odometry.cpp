/// @file lidar_odometry.cpp
/// @brief Implementation of LiDAR scan-to-scan GICP registration.

#include "lidar_odometry.hpp"

#include <small_gicp/registration/registration_helper.hpp>

#include <cstring>
#include <vector>

using namespace gtsam;

// ─── Point conversion ────────────────────────────────────────────────────────

/// @brief Convert foxglove::PointCloud to Eigen points for GICP.
static std::vector<Eigen::Vector4f> toEigenPoints(
    const foxglove::PointCloud& cloud) {
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

/// @brief Convert Eigen::Isometry3d to gtsam::Pose3.
static Pose3 isometryToPose3(const Eigen::Isometry3d& T) {
    return Pose3(Rot3(T.rotation()), Point3(T.translation()));
}

// ─── GICP odometry ───────────────────────────────────────────────────────────

std::vector<LidarRelativePose> computeLidarOdometry(
    const std::vector<Stamped<foxglove::PointCloud>>& lidar_scans,
    const slam::geometry::Transform<Pose3, BodyFrame, LidarTopFrame>& body_from_lidar) {

    std::vector<LidarRelativePose> results;
    if (lidar_scans.size() < 2) return results;

    const auto lidar_from_body = body_from_lidar.inverse();

    small_gicp::RegistrationSetting setting;
    setting.num_threads = 4;
    setting.downsampling_resolution = 0.5;
    setting.max_correspondence_distance = 2.0;

    auto prev_points = toEigenPoints(lidar_scans[0].msg);
    auto [prev_cloud, prev_tree] = small_gicp::preprocess_points(
        prev_points, setting.downsampling_resolution, 10, setting.num_threads);

    for (std::size_t i = 1; i < lidar_scans.size(); ++i) {
        auto curr_points = toEigenPoints(lidar_scans[i].msg);
        auto [curr_cloud, curr_tree] = small_gicp::preprocess_points(
            curr_points, setting.downsampling_resolution, 10, setting.num_threads);

        auto reg_result = small_gicp::align(
            *prev_cloud, *curr_cloud, *prev_tree,
            Eigen::Isometry3d::Identity(), setting);

        using LidarTf = slam::geometry::Transform<Pose3, LidarTopFrame, LidarTopFrame>;
        using BodyTf  = slam::geometry::Transform<Pose3, BodyFrame, BodyFrame>;

        LidarTf T_lidar(isometryToPose3(reg_result.T_target_source));
        BodyTf  T_body = body_from_lidar * T_lidar * lidar_from_body;

        LidarRelativePose lrp;
        lrp.stamp_from  = lidar_scans[i - 1].stamp.value();
        lrp.stamp_to    = lidar_scans[i].stamp.value();
        lrp.T_lidar     = T_lidar;
        lrp.T_body      = T_body;
        lrp.converged   = reg_result.converged;
        lrp.error       = reg_result.error;
        lrp.num_inliers = reg_result.num_inliers;
        results.push_back(lrp);

        prev_cloud = curr_cloud;
        prev_tree  = curr_tree;
    }
    return results;
}
