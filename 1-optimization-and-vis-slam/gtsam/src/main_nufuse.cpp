/// @file main.cpp
/// @brief GPS + IMU + LiDAR factor graph from NuScenes MCAP data.
///
/// Pipeline:
///   1. Load MCAP -> typed sensor measurements.
///   2. Compute LiDAR GICP odometry -> body-frame relative poses.
///   3. Feed measurements into processor -> FactorStorage.
///   4. Build GTSAM graph -> optimize -> type-safe results.

#include "domain/scene_data.hpp"
#include "graph/builder.hpp"
#include "io/mcap_loader.hpp"
#include "processing/lidar_odometry.hpp"
#include "processing/measurement_processor.hpp"
#include "results/optimizer.hpp"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <unordered_map>

namespace {

using namespace nufuse;

// ─── Validation ──────────────────────────────────────────────────────────────

bool validateScene(const domain::SceneData& scene) {
    if (scene.imu.empty() || scene.gnss.size() < 2 || scene.odom.empty()) {
        std::cerr << "Insufficient data (need IMU, >=2 GNSS, odom).\n";
        return false;
    }
    if (!scene.extrinsics.body_from_lidar_top) {
        std::cerr << "Missing body_from_lidar_top extrinsic!\n";
        return false;
    }
    return true;
}

// ─── LiDAR relative pose map ────────────────────────────────────────────────

std::unordered_map<uint64_t, gtsam::Pose3> buildLidarRelMap(
    const std::vector<processing::LidarRelativePose>& lidar_poses) {
    std::unordered_map<uint64_t, gtsam::Pose3> map;
    for (const auto& lp : lidar_poses) {
        if (lp.converged) {
            map[lp.stamp_to.value()] = lp.T_lidar.value();
        }
    }
    return map;
}

// ─── Measurement merging (chronological interleaving) ────────────────────────

graph::FactorStorage processMeasurements(
    const domain::SceneData& scene,
    const std::unordered_map<uint64_t, gtsam::Pose3>& lidar_rel) {

    processing::ProcessorConfig cfg;
    processing::MeasurementProcessor processor(cfg, scene.odom);

    auto gnss_it  = scene.gnss.cbegin();
    auto imu_it   = scene.imu.cbegin();
    auto lidar_it = scene.lidar.cbegin();

    while (gnss_it != scene.gnss.cend() ||
           imu_it != scene.imu.cend() ||
           lidar_it != scene.lidar.cend()) {

        const uint64_t t_gnss  = gnss_it  != scene.gnss.cend()  ? gnss_it->stamp.value()  : UINT64_MAX;
        const uint64_t t_imu   = imu_it   != scene.imu.cend()   ? imu_it->stamp.value()   : UINT64_MAX;
        const uint64_t t_lidar = lidar_it != scene.lidar.cend() ? lidar_it->stamp.value() : UINT64_MAX;

        if (t_imu <= t_gnss && t_imu <= t_lidar) {
            processor.addImu(*imu_it++);
        } else if (t_gnss <= t_lidar) {
            processor.addGnss(*gnss_it++);
        } else {
            const auto stamp = lidar_it->stamp;
            auto it = lidar_rel.find(stamp.value());
            processor.addLidar(stamp, it != lidar_rel.end()
                ? std::optional<gtsam::Pose3>(it->second) : std::nullopt);
            ++lidar_it;
        }
    }

    return std::move(processor).finalize();
}

// ─── Results printing ────────────────────────────────────────────────────────

void printTrajectory(const results::OptimizedResults& res) {
    std::cout << "\nOptimized ENU trajectory (first/last 3) [m]:\n";
    const int n = res.num_keyframes;
    for (int i = 0; i <= std::min(2, n); ++i) {
        const auto t = res.poses[i].pose.value().translation();
        std::cout << "  pose[" << i << "]:  E=" << t.x()
                  << "  N=" << t.y() << "  U=" << t.z() << "\n";
    }
    if (n > 5) std::cout << "  ...\n";
    for (int i = std::max(3, n - 2); i <= n; ++i) {
        const auto t = res.poses[i].pose.value().translation();
        std::cout << "  pose[" << i << "]:  E=" << t.x()
                  << "  N=" << t.y() << "  U=" << t.z() << "\n";
    }
}

void printBias(const results::OptimizedResults& res) {
    const auto& bias = res.biases.back().bias;
    std::cout << "\nFinal bias: accel=[" << bias.accelerometer().transpose()
              << "]  gyro=[" << bias.gyroscope().transpose() << "]\n";
}

void printError(const results::OptimizedResults& res,
                const domain::SceneData& scene) {
    const gtsam::Point3 gt = scene.odom.back().pose.value().translation()
                           - scene.odom.front().pose.value().translation();
    const gtsam::Point3 est = res.poses.back().pose.value().translation();
    const gtsam::Point3 delta = est - gt;

    std::cout << "\nEnd-position error:\n"
              << "  Ground truth : " << gt.transpose() << " [m]\n"
              << "  Estimated    : " << est.transpose() << " [m]\n"
              << "  ||error||    : " << delta.norm() << " m\n";
}

void printLidarExtrinsics(const results::OptimizedResults& res,
                          const domain::SceneData& scene) {
    std::cout << "\nLidar extrinsics estimation:\n"
              << "  Estimated T_body_lidar:\n" << res.body_from_lidar.value() << "\n"
              << "  GT T_body_lidar:\n"
              << scene.extrinsics.body_from_lidar_top->value() << "\n";
}

}  // anonymous namespace

// ─── main ────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <path-to-mcap>\n";
        return EXIT_FAILURE;
    }

    const std::filesystem::path mcap_path(argv[1]);
    if (!std::filesystem::exists(mcap_path)) {
        std::cerr << "File not found: " << mcap_path << "\n";
        return EXIT_FAILURE;
    }

    // 1. Load
    std::cout << "Loading " << mcap_path.filename() << " ...\n";
    nufuse::domain::SceneData scene;
    try {
        scene = nufuse::io::loadMcap(mcap_path);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return EXIT_FAILURE;
    }

    if (!validateScene(scene)) return EXIT_FAILURE;

    std::cout << "  IMU: " << scene.imu.size()
              << "  GNSS: " << scene.gnss.size()
              << "  Odom: " << scene.odom.size()
              << "  LiDAR: " << scene.lidar.size() << "\n";

    // 2. LiDAR odometry
    std::cout << "\nComputing LiDAR GICP odometry...\n";
    auto lidar_poses = nufuse::processing::computeLidarOdometry(
        scene.lidar, *scene.extrinsics.body_from_lidar_top);
    std::cout << "  LiDAR pairs: " << lidar_poses.size()
              << "  converged: "
              << std::count_if(lidar_poses.begin(), lidar_poses.end(),
                               [](const auto& lp) { return lp.converged; })
              << "\n";

    auto lidar_rel = buildLidarRelMap(lidar_poses);

    // 3. Process measurements
    auto storage = processMeasurements(scene, lidar_rel);

    // 4. Build graph & optimize
    auto gtsam_graph = nufuse::graph::buildGraph(storage);
    std::cout << "Graph: " << gtsam_graph.graph.size() << " factors, "
              << gtsam_graph.initial.size() << " variables, "
              << gtsam_graph.num_keyframes + 1 << " keyframes, "
              << gtsam_graph.num_corrupted << " corrupted GNSS fixes\n";

    auto results = nufuse::results::optimize(gtsam_graph, storage);

    // 5. Print
    printTrajectory(results);
    printBias(results);
    printError(results, scene);
    printLidarExtrinsics(results, scene);

    return EXIT_SUCCESS;
}
