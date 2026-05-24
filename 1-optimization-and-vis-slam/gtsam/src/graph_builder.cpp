/// @file graph_builder.cpp
/// @brief Implementation of buildGtsamGraph.

#include "graph_builder.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include "lidar_extrinsics_factor.hpp"

using namespace gtsam;
using symbol_shorthand::B;
using symbol_shorthand::V;
using symbol_shorthand::X;

Symbol T_BL_key('L', 0);  // Static extrinsic: base_link → lidar (assumed constant).

// ─── Noise models ────────────────────────────────────────────────────────────

struct NoiseModels {
    SharedNoiseModel pose;
    SharedNoiseModel velocity;
    SharedNoiseModel bias;
    SharedNoiseModel gps;
    SharedNoiseModel lidar;
};

static NoiseModels makeNoiseModels() {
    NoiseModels nm;
    nm.pose = noiseModel::Diagonal::Sigmas(
        (Vector(6) << 0.05, 0.05, 0.05, 0.5, 0.5, 0.5).finished());
    nm.velocity = noiseModel::Isotropic::Sigma(3, 0.1);
    nm.bias = noiseModel::Diagonal::Sigmas(
        (Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished());
    auto base  = noiseModel::Isotropic::Sigma(3, 1.0);
    auto huber = noiseModel::mEstimator::Huber::Create(1.345);
    nm.gps = noiseModel::Robust::Create(huber, base);
    nm.lidar = noiseModel::Diagonal::Sigmas(
        (Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished());
    return nm;
}

// ─── Graph construction ──────────────────────────────────────────────────────

GtsamGraph buildGtsamGraph(const FactorStorage& storage) {
    const auto noise = makeNoiseModels();
    GtsamGraph result;
    auto& graph  = result.graph;
    auto& values = result.initial;

    // 1. Prior factors.
    if (storage.prior) {
        const auto& p   = *storage.prior;
        const int   idx = p.keyframe.value();
        graph.addPrior<Pose3>(X(idx), p.pose.value(), noise.pose);
        graph.addPrior<Vector3>(V(idx), p.velocity.value(), noise.velocity);
        graph.addPrior<imuBias::ConstantBias>(B(idx), p.bias, noise.bias);
        graph.add(GPSFactor(X(idx), p.gps_position.value(), noise.gps));
    }

    // 2. IMU factors.
    for (const auto& f : storage.imu_factors) {
        const int i = f.from.value();
        const int j = f.to.value();
        graph.add(CombinedImuFactor(X(i), V(i), X(j), V(j), B(i), B(j), f.pim));
    }

    // 3. GPS factors.
    for (const auto& f : storage.gps_factors) {
        const int idx = f.keyframe.value();
        graph.add(GPSFactor(X(idx), f.position_enu.value(), noise.gps));
    }

    // 4. LiDAR between-factors.
    for (const auto& f : storage.lidar_factors) {
        // const int i = f.from.value();
        // const int j = f.to.value();
        // graph.add(BetweenFactor<Pose3>(X(i), X(j), f.relative_pose.value(), noise.lidar));
        const int i = f.from.value();
        const int j = f.to.value();
        graph.add(LidarExtrinsicsFactor(X(i), X(j), T_BL_key, f.relative_pose.value(), noise.lidar));
    }

    // 5. Initial values from keyframe estimates.
    values.insert(T_BL_key, Pose3::Identity());
    for (const auto& est : storage.estimates) {
        const int idx = est.id.value();
        values.insert(X(idx), est.pose.value());
        values.insert(V(idx), est.velocity.value());
        values.insert(B(idx), est.bias);
        result.num_keyframes = std::max(result.num_keyframes, idx);
    }

    result.num_corrupted = storage.num_corrupted;

    return result;
}
