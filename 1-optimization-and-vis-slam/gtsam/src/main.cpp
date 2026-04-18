#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include <numbers>

/// @brief Build a small 2D pose-graph, optimize it, and print the results.
///
/// Graph layout (square with loop closure):
///
///   1 --odom--> 2 --odom--> 3
///               ^            |
///               |           odom
///            loop            |
///               |            v
///               5 <--odom-- 4
///
/// @return 0 on success.
int main() {
    using namespace gtsam;

    // ---- Factor graph ----
    NonlinearFactorGraph graph;

    const auto prior_noise =
        noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
    graph.addPrior(1, Pose2(0.0, 0.0, 0.0), prior_noise);

    const auto odom_noise =
        noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
    const double half_pi = std::numbers::pi / 2.0;

    graph.emplace_shared<BetweenFactor<Pose2>>(1, 2, Pose2(2.0, 0.0, 0.0), odom_noise);
    graph.emplace_shared<BetweenFactor<Pose2>>(2, 3, Pose2(2.0, 0.0, half_pi), odom_noise);
    graph.emplace_shared<BetweenFactor<Pose2>>(3, 4, Pose2(2.0, 0.0, half_pi), odom_noise);
    graph.emplace_shared<BetweenFactor<Pose2>>(4, 5, Pose2(2.0, 0.0, half_pi), odom_noise);

    // Loop closure: 5 -> 2
    const auto loop_noise =
        noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0.2));
    graph.emplace_shared<BetweenFactor<Pose2>>(5, 2, Pose2(2.0, 0.0, half_pi), loop_noise);

    graph.print("Factor graph:\n");

    // ---- Initial estimates (deliberately noisy) ----
    Values initial;
    initial.insert(1, Pose2(0.5,  0.0,  0.2));
    initial.insert(2, Pose2(2.3,  0.1, -0.2));
    initial.insert(3, Pose2(4.1,  0.1,  half_pi));
    initial.insert(4, Pose2(3.9,  2.1,  std::numbers::pi));
    initial.insert(5, Pose2(2.1,  2.1, -half_pi));

    // ---- Optimize ----
    LevenbergMarquardtOptimizer optimizer(graph, initial);
    const Values result = optimizer.optimize();

    result.print("Optimized poses:\n");

    // ---- Marginal covariances ----
    Marginals marginals(graph, result);
    std::cout << "\nMarginal covariance on pose 1:\n"
              << marginals.marginalCovariance(1) << '\n';

    return 0;
}
