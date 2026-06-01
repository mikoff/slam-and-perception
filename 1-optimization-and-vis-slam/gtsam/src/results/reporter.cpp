/// @file results/reporter.cpp
/// @brief Implementation of console result reporting.

#include "results/reporter.hpp"

#include <iostream>

namespace nufuse::results {

void printTrajectory(const OptimizedResults& res) {
  std::cout << "\nOptimized ENU trajectory (first/last 3) [m]:\n";
  const int num_keyframes = res.num_keyframes;
  for (int i = 0; i <= std::min(2, num_keyframes); ++i) {
    const auto translation = res.poses[i].pose.value().translation();
    std::cout << "  pose[" << i << "]:  E=" << translation.x() << "  N=" << translation.y()
              << "  U=" << translation.z() << "\n";
  }
  if (num_keyframes > 5) std::cout << "  ...\n";
  for (int i = std::max(3, num_keyframes - 2); i <= num_keyframes; ++i) {
    const auto translation = res.poses[i].pose.value().translation();
    std::cout << "  pose[" << i << "]:  E=" << translation.x() << "  N=" << translation.y()
              << "  U=" << translation.z() << "\n";
  }
}

void printBias(const OptimizedResults& res) {
  const auto& bias = res.biases.back().bias;
  std::cout << "\nFinal bias: accel=[" << bias.accelerometer().transpose() << "]  gyro=["
            << bias.gyroscope().transpose() << "]\n";
}

void printError(const OptimizedResults& res, const domain::SceneData& scene, bool init_from_gt) {
  const gtsam::Point3 ground_truth =
      scene.odom.back().pose.value().translation() - scene.odom.front().pose.value().translation();
  const gtsam::Point3 odom_start = scene.odom.front().pose.value().translation();
  const gtsam::Point3 estimated = res.poses.back().pose.value().translation();

  const gtsam::Point3 estimated_displacement = init_from_gt ? (estimated - odom_start) : estimated;
  const gtsam::Point3 error = estimated_displacement - ground_truth;

  std::cout << "\nEnd-position error:\n"
            << "  Ground truth : " << ground_truth.transpose() << " [m]\n"
            << "  Estimated    : " << estimated_displacement.transpose() << " [m]\n"
            << "  ||error||    : " << error.norm() << " m\n";
}

void printLidarExtrinsics(const OptimizedResults& res, const domain::SceneData& scene) {
  if (!scene.extrinsics.body_from_lidar_top) {
    std::cout << "\nLidar extrinsics: not available (lidar disabled)\n";
    return;
  }
  std::cout << "\nLidar extrinsics estimation:\n"
            << "  Estimated T_body_lidar:\n"
            << res.body_from_lidar.value() << "\n"
            << "  GT T_body_lidar:\n"
            << scene.extrinsics.body_from_lidar_top->value() << "\n";
}

void printRadarExtrinsics(const OptimizedResults& res, const domain::SceneData& scene) {
  if (res.radar_extrinsics.empty()) return;
  std::cout << "\nRadar extrinsics estimation:\n";

  auto printOne = [&](core::RadarSensorId id, const char* name, const auto& gt_opt) {
    auto it = res.radar_extrinsics.find(id);
    if (it == res.radar_extrinsics.end()) return;
    std::cout << "  " << name << ":\n"
              << "    Estimated: t=" << it->second.translation().transpose() << "\n";
    if (gt_opt) {
      std::cout << "    GT:        t=" << gt_opt->value().translation().transpose() << "\n";
    }
  };

  printOne(core::RadarSensorId::Front, "RADAR_FRONT", scene.extrinsics.body_from_radar_front);
  printOne(core::RadarSensorId::FrontLeft, "RADAR_FRONT_LEFT",
           scene.extrinsics.body_from_radar_front_left);
  printOne(core::RadarSensorId::FrontRight, "RADAR_FRONT_RIGHT",
           scene.extrinsics.body_from_radar_front_right);
  printOne(core::RadarSensorId::BackLeft, "RADAR_BACK_LEFT",
           scene.extrinsics.body_from_radar_back_left);
  printOne(core::RadarSensorId::BackRight, "RADAR_BACK_RIGHT",
           scene.extrinsics.body_from_radar_back_right);
}

}  // namespace nufuse::results
