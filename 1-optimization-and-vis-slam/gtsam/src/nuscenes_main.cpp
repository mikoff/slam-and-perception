/// @file nuscenes_main.cpp
/// @brief Loads NuScenes MCAP data and prints a summary of all sensor streams.

#include "nuscenes_mcap_loader.hpp"

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <gtsam/navigation/CombinedImuFactor.h>

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <path-to-mcap>" << std::endl;
        return EXIT_FAILURE;
    }

    const std::filesystem::path mcap_path(argv[1]);
    if (!std::filesystem::exists(mcap_path)) {
        std::cerr << "File not found: " << mcap_path << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "Loading " << mcap_path.filename() << " ..." << std::endl;
    nuscenes::SceneData scene;
    try {
        scene = nuscenes::loadMcap(mcap_path);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "Loaded: "
              << scene.imu.size() << " IMU, "
              << scene.odom.size() << " odom, "
              << scene.gnss.size() << " GNSS, "
              << scene.lidar.size() << " lidar, "
              << scene.radar.size() << " radar, "
              << scene.images.size() << " image, "
              << scene.ego_poses.size() << " ego poses" << std::endl;

    if (!scene.imu.empty()) {
        const auto& m = scene.imu.front();
        const auto& a = m.linear_acceleration.value();
        const auto& g = m.angular_velocity.value();
        std::cout << "\nFirst IMU (ImuFrame): "
                  << "accel=(" << a(0) << ", " << a(1) << ", " << a(2) << ") "
                  << "gyro=(" << g(0) << ", " << g(1) << ", " << g(2) << ")"
                  << std::endl;
    }
    if (!scene.odom.empty()) {
        const auto& m = scene.odom.front();
        const auto& t = m.pose.value().translation();
        const auto& v = m.velocity.value();
        std::cout << "First odom (BodyFrame->MapFrame): "
                  << "pos=(" << t(0) << ", " << t(1) << ", " << t(2) << ") "
                  << "vel=(" << v(0) << ", " << v(1) << ", " << v(2) << ")"
                  << std::endl;
    }
    if (!scene.gnss.empty()) {
        const auto& m = scene.gnss.front();
        const auto& lla = m.lla.value();
        std::cout << "First GNSS (WGS84): "
                  << "lat=" << lla(0) << " lon=" << lla(1)
                  << " alt=" << lla(2) << std::endl;
    }
    if (!scene.lidar.empty()) {
        const auto& m = scene.lidar.front();
        std::cout << "First LiDAR (frame=" << m.msg.frame_id() << "): "
                  << m.msg.data().size() << " bytes, "
                  << "stride=" << m.msg.point_stride() << std::endl;
    }
    if (!scene.radar.empty()) {
        const auto& m = scene.radar.front();
        std::cout << "First radar (frame=" << m.msg.frame_id() << "): "
                  << m.msg.data().size() << " bytes, "
                  << "stride=" << m.msg.point_stride() << std::endl;
    }
    if (!scene.images.empty()) {
        const auto& m = scene.images.front();
        std::cout << "First image (frame=" << m.msg.frame_id() << "): "
                  << "format=" << m.msg.format()
                  << " size=" << m.msg.data().size() << " bytes" << std::endl;
    }
    // ---- Extrinsic calibration summary ------------------------------------
    std::cout << "\nExtrinsic calibration (body -> sensor):" << std::endl;
    const auto& cal = scene.extrinsics;
    auto printExtrinsic = [](const char* name, const auto& opt) {
        if (opt) {
            const auto& t = opt->value().translation();
            std::cout << "  " << name << ": t=("
                      << t(0) << ", " << t(1) << ", " << t(2) << ")"
                      << std::endl;
        }
    };
    printExtrinsic("LIDAR_TOP       ", cal.body_from_lidar_top);
    printExtrinsic("CAM_FRONT       ", cal.body_from_cam_front);
    printExtrinsic("CAM_FRONT_LEFT  ", cal.body_from_cam_front_left);
    printExtrinsic("CAM_FRONT_RIGHT ", cal.body_from_cam_front_right);
    printExtrinsic("CAM_BACK        ", cal.body_from_cam_back);
    printExtrinsic("CAM_BACK_LEFT   ", cal.body_from_cam_back_left);
    printExtrinsic("CAM_BACK_RIGHT  ", cal.body_from_cam_back_right);
    printExtrinsic("RADAR_FRONT     ", cal.body_from_radar_front);
    printExtrinsic("RADAR_FRONT_LEFT", cal.body_from_radar_front_left);
    printExtrinsic("RADAR_FRONT_RIGHT", cal.body_from_radar_front_right);
    printExtrinsic("RADAR_BACK_LEFT ", cal.body_from_radar_back_left);
    printExtrinsic("RADAR_BACK_RIGHT", cal.body_from_radar_back_right);

    if (!scene.ego_poses.empty()) {
        const auto& first = scene.ego_poses.front().msg.value().translation();
        const auto& last = scene.ego_poses.back().msg.value().translation();
        std::cout << "\nEgo trajectory (map -> body): "
                  << scene.ego_poses.size() << " poses, "
                  << "start=(" << first(0) << ", " << first(1) << ", " << first(2) << ") "
                  << "end=(" << last(0) << ", " << last(1) << ", " << last(2) << ")"
                  << std::endl;
    }

    return EXIT_SUCCESS;


}
