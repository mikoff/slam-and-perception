/// @file graph/builder.cpp
/// @brief Implementation of graph construction.

#include "graph/builder.hpp"
#include "factor/lidar_extrinsics.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/slam/BetweenFactor.h>

namespace nufuse::graph {

using namespace gtsam;
using symbol_shorthand::B;
using symbol_shorthand::V;
using symbol_shorthand::X;

static const Symbol kLidarExtrinsicKey('L', 0);

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
    nm.gps   = noiseModel::Robust::Create(huber, base);
    nm.lidar = noiseModel::Diagonal::Sigmas(
        (Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished());
    return nm;
}

// ─── Timestamp to index mapping ─────────────────────────────────────────────

static std::unordered_map<uint64_t, int> buildStampIndex(
    const FactorStorage& storage) {
    std::unordered_map<uint64_t, int> map;
    for (int i = 0; i < static_cast<int>(storage.estimates.size()); ++i) {
        map[storage.estimates[i].stamp.value()] = i;
    }
    return map;
}

// ─── Factor insertion helpers ────────────────────────────────────────────────

static void addPriorFactors(NonlinearFactorGraph& graph,
                            const PriorFactor& p, int idx,
                            const NoiseModels& noise) {
    graph.addPrior<Pose3>(X(idx), p.pose.value(), noise.pose);
    graph.addPrior<Vector3>(V(idx), p.velocity.value(), noise.velocity);
    graph.addPrior<imuBias::ConstantBias>(B(idx), p.bias, noise.bias);
    graph.add(GPSFactor(X(idx), p.gps_position.value(), noise.gps));
}

static void addImuFactors(NonlinearFactorGraph& graph,
                          const std::vector<StoredImuFactor>& factors,
                          const std::unordered_map<uint64_t, int>& stamp_idx) {
    for (const auto& f : factors) {
        const int i = stamp_idx.at(f.stamp_from.value());
        const int j = stamp_idx.at(f.stamp_to.value());
        graph.add(CombinedImuFactor(X(i), V(i), X(j), V(j), B(i), B(j), f.pim));
    }
}

static void addGpsFactors(NonlinearFactorGraph& graph,
                          const std::vector<StoredGpsFactor>& factors,
                          const std::unordered_map<uint64_t, int>& stamp_idx,
                          const NoiseModels& noise) {
    for (const auto& f : factors) {
        const int idx = stamp_idx.at(f.stamp.value());
        graph.add(GPSFactor(X(idx), f.position_enu.value(), noise.gps));
    }
}

static void addLidarFactors(NonlinearFactorGraph& graph,
                            const std::vector<StoredLidarFactor>& factors,
                            const std::unordered_map<uint64_t, int>& stamp_idx,
                            const NoiseModels& noise) {
    for (const auto& f : factors) {
        const int i = stamp_idx.at(f.stamp_from.value());
        const int j = stamp_idx.at(f.stamp_to.value());
        graph.add(factor::LidarExtrinsicsFactor(
            X(i), X(j), kLidarExtrinsicKey, f.relative_pose.value(), noise.lidar));
    }
}

static void insertInitialValues(Values& values,
                                const std::vector<KeyframeEstimate>& estimates) {
    values.insert(kLidarExtrinsicKey, Pose3::Identity());
    for (int i = 0; i < static_cast<int>(estimates.size()); ++i) {
        const auto& est = estimates[i];
        values.insert(X(i), est.pose.value());
        values.insert(V(i), est.velocity.value());
        values.insert(B(i), est.bias);
    }
}

// ─── Public interface ────────────────────────────────────────────────────────

GtsamGraph buildGraph(const FactorStorage& storage) {
    const auto noise = makeNoiseModels();
    GtsamGraph result;
    result.stamp_to_index = buildStampIndex(storage);

    if (storage.prior) {
        const int idx = result.stamp_to_index.at(storage.prior->stamp.value());
        addPriorFactors(result.graph, *storage.prior, idx, noise);
    }

    addImuFactors(result.graph, storage.imu_factors, result.stamp_to_index);
    addGpsFactors(result.graph, storage.gps_factors, result.stamp_to_index, noise);
    addLidarFactors(result.graph, storage.lidar_factors, result.stamp_to_index, noise);
    insertInitialValues(result.initial, storage.estimates);

    result.num_keyframes = static_cast<int>(storage.estimates.size()) - 1;
    result.num_corrupted = storage.num_corrupted;
    return result;
}

}  // namespace nufuse::graph
