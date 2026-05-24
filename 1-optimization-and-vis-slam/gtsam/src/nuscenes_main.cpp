/// @file nuscenes_main.cpp
/// @brief GPS + IMU + LiDAR factor graph from NuScenes MCAP data.
///
/// Pipeline:
///   1. Load MCAP → raw sensor measurements.
///   2. Compute LiDAR GICP odometry → body-frame relative poses.
///   3. Feed IMU / GNSS / LiDAR into MeasurementProcessor → FactorStorage.
///   4. Build GTSAM graph from FactorStorage → optimize → print results.

#include "graph_builder.hpp"
#include "lidar_odometry.hpp"
#include "measurement_processor.hpp"
#include "nuscenes_mcap_loader.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <unordered_map>

using namespace gtsam;
using symbol_shorthand::B;
using symbol_shorthand::X;

// ─── Optimization ────────────────────────────────────────────────────────────

static Values optimize(const NonlinearFactorGraph& graph, const Values& initial) {
    LevenbergMarquardtParams params;
    params.setVerbosityLM("SUMMARY");
    LevenbergMarquardtOptimizer optimizer(graph, initial, params);
    return optimizer.optimize();
}

// ─── Results ─────────────────────────────────────────────────────────────────

static void printResults(const Values& result, int num_keyframes,
                         const nuscenes::SceneData& scene) {
    std::cout << "\nOptimized ENU trajectory (first/last 3) [m]:\n";
    for (int i = 0; i <= std::min(2, num_keyframes); ++i) {
        const auto t = result.at<Pose3>(X(i)).translation();
        std::cout << "  pose[" << i << "]:  E=" << t.x()
                  << "  N=" << t.y() << "  U=" << t.z() << "\n";
    }
    if (num_keyframes > 5) std::cout << "  ...\n";
    for (int i = std::max(3, num_keyframes - 2); i <= num_keyframes; ++i) {
        const auto t = result.at<Pose3>(X(i)).translation();
        std::cout << "  pose[" << i << "]:  E=" << t.x()
                  << "  N=" << t.y() << "  U=" << t.z() << "\n";
    }

    const auto bias = result.at<imuBias::ConstantBias>(B(num_keyframes));
    std::cout << "\nFinal bias: accel=[" << bias.accelerometer().transpose()
              << "]  gyro=[" << bias.gyroscope().transpose() << "]\n";

    const Point3 gt = scene.odom.back().pose.value().translation()
                    - scene.odom.front().pose.value().translation();
    const Point3 est = result.at<Pose3>(X(num_keyframes)).translation();
    const Point3 delta = est - gt;

    std::cout << "\nEnd-position error:\n"
              << "  Ground truth : " << gt.transpose() << " [m]\n"
              << "  Estimated    : " << est.transpose() << " [m]\n"
              << "  ||error||    : " << delta.norm() << " m\n";

    Symbol T_BL_key('L', 0);  // Static extrinsic: base_link → lidar (assumed constant).
    std::cout << "\nLidar extrinsics estimation:\n";
    if (result.exists(T_BL_key)) {
        const auto T_BL = result.at<Pose3>(T_BL_key);
        std::cout << "  Estimated T_body_lidar:\n" << T_BL << "\n";
        std::cout << "  GT: T_body_lidar:\n" << scene.extrinsics.body_from_lidar_top->value() << "\n";
    } else {
        std::cout << "  No lidar extrinsics factor in graph.\n";
    }
}

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

    // ── 1. Load raw measurements ─────────────────────────────────────────────
    std::cout << "Loading " << mcap_path.filename() << " ...\n";
    nuscenes::SceneData scene;
    try {
        scene = nuscenes::loadMcap(mcap_path);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return EXIT_FAILURE;
    }

    if (scene.imu.empty() || scene.gnss.size() < 2 || scene.odom.empty()) {
        std::cerr << "Insufficient data (need IMU, >=2 GNSS, odom).\n";
        return EXIT_FAILURE;
    }

    std::cout << "  IMU: " << scene.imu.size()
              << "  GNSS: " << scene.gnss.size()
              << "  Odom: " << scene.odom.size()
              << "  LiDAR: " << scene.lidar.size() << "\n";

    // ── 2. Compute LiDAR GICP odometry ───────────────────────────────────────
    if (!scene.extrinsics.body_from_lidar_top) {
        std::cerr << "Missing body_from_lidar_top extrinsic!\n";
        return EXIT_FAILURE;
    }
    std::cout << "\nComputing LiDAR GICP odometry...\n";
    auto lidar_poses = computeLidarOdometry(
        scene.lidar, *scene.extrinsics.body_from_lidar_top);
    std::cout << "  LiDAR pairs: " << lidar_poses.size()
              << "  converged: "
              << std::count_if(lidar_poses.begin(), lidar_poses.end(),
                               [](const auto& lp) { return lp.converged; })
              << "\n";

    // Build stamp_to → body-frame relative pose map (converged pairs only).
    std::unordered_map<uint64_t, Pose3> lidar_rel;
    for (const auto& lp : lidar_poses)
        if (lp.converged) lidar_rel[lp.stamp_to] = lp.T_lidar.value();

    // ── 3. Process measurements → FactorStorage ──────────────────────────────
    ProcessorConfig cfg;
    MeasurementProcessor processor(cfg, scene.odom);

    auto gnss_it  = scene.gnss.cbegin();
    auto imu_it   = scene.imu.cbegin();
    auto lidar_it = scene.lidar.cbegin();

    while (gnss_it  != scene.gnss.cend()  ||
           imu_it   != scene.imu.cend()   ||
           lidar_it != scene.lidar.cend()) {
        const uint64_t t_gnss  = gnss_it  != scene.gnss.cend()  ? gnss_it->stamp.value()  : UINT64_MAX;
        const uint64_t t_imu   = imu_it   != scene.imu.cend()   ? imu_it->stamp.value()   : UINT64_MAX;
        const uint64_t t_lidar = lidar_it != scene.lidar.cend() ? lidar_it->stamp.value() : UINT64_MAX;

        if (t_imu <= t_gnss && t_imu <= t_lidar) {
            processor.addImu(*imu_it++);
        } else if (t_gnss <= t_lidar) {
            processor.addGnss(*gnss_it++);
        } else {
            const uint64_t stamp = lidar_it->stamp.value();
            auto it = lidar_rel.find(stamp);
            processor.addLidar(stamp, it != lidar_rel.end()
                ? std::optional<Pose3>(it->second) : std::nullopt);
            ++lidar_it;
        }
    }

    FactorStorage storage = std::move(processor).finalize();

    // ── 4. Build GTSAM graph → optimize ──────────────────────────────────────
    auto [graph, initial, num_keyframes, num_corrupted] = buildGtsamGraph(storage);
    std::cout << "Graph: " << graph.size() << " factors, "
              << initial.size() << " variables, "
              << num_keyframes + 1 << " keyframes, "
              << num_corrupted << " corrupted GNSS fixes\n";

    auto result = optimize(graph, initial);
    printResults(result, num_keyframes, scene);

    return EXIT_SUCCESS;
}
