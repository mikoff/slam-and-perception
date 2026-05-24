/// @file results/optimizer.cpp
/// @brief Implementation of optimization and results unpacking.

#include "results/optimizer.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

namespace nufuse::results {

using namespace gtsam;
using symbol_shorthand::B;
using symbol_shorthand::V;
using symbol_shorthand::X;

static const Symbol kLidarExtrinsicKey('L', 0);

// ─── LM optimization ────────────────────────────────────────────────────────

static Values runLM(const NonlinearFactorGraph& graph, const Values& initial) {
    LevenbergMarquardtParams params;
    params.setVerbosityLM("SUMMARY");
    LevenbergMarquardtOptimizer optimizer(graph, initial, params);
    return optimizer.optimize();
}

// ─── Covariance extraction ───────────────────────────────────────────────────

static void extractPosesAndVelocities(
    const Values& values, const Marginals& marginals,
    const graph::FactorStorage& storage, OptimizedResults& out) {

    for (int i = 0; i < static_cast<int>(storage.estimates.size()); ++i) {
        OptimizedPose op;
        op.stamp = storage.estimates[i].stamp;
        op.pose = core::BodyInEnu(values.at<Pose3>(X(i)));
        op.covariance = marginals.marginalCovariance(X(i));
        out.poses.push_back(op);

        OptimizedVelocity ov;
        ov.stamp = storage.estimates[i].stamp;
        ov.velocity = core::EnuVelocity(values.at<Vector3>(V(i)));
        ov.covariance = marginals.marginalCovariance(V(i));
        out.velocities.push_back(ov);

        OptimizedBias ob;
        ob.stamp = storage.estimates[i].stamp;
        ob.bias = values.at<imuBias::ConstantBias>(B(i));
        ob.covariance = marginals.marginalCovariance(B(i));
        out.biases.push_back(ob);
    }
}

static void extractLidarExtrinsics(const Values& values,
                                   const Marginals& marginals,
                                   OptimizedResults& out) {
    if (values.exists(kLidarExtrinsicKey)) {
        out.body_from_lidar = core::BodyFromLidar(values.at<Pose3>(kLidarExtrinsicKey));
        out.lidar_extrinsics_covariance = marginals.marginalCovariance(kLidarExtrinsicKey);
    } else {
        out.body_from_lidar = core::BodyFromLidar(Pose3::Identity());
        out.lidar_extrinsics_covariance.setZero();
    }
}

// ─── Public interface ────────────────────────────────────────────────────────

OptimizedResults optimize(const graph::GtsamGraph& gtsam_graph,
                          const graph::FactorStorage& storage) {
    Values values = runLM(gtsam_graph.graph, gtsam_graph.initial);
    Marginals marginals(gtsam_graph.graph, values);

    OptimizedResults out;
    out.num_keyframes = gtsam_graph.num_keyframes;
    out.num_corrupted_gnss = gtsam_graph.num_corrupted;

    extractPosesAndVelocities(values, marginals, storage, out);
    extractLidarExtrinsics(values, marginals, out);

    return out;
}

}  // namespace nufuse::results
