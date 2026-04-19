/// @file main.cpp
/// @brief 2-D pose-graph example: build, optimize, and inspect marginals.
///
/// Graph layout (square with loop closure):
///
///   1 --odom--> 2 --odom--> 3
///               ^            |
///               |           odom
///            loop            |
///               |            v
///               5 <--odom-- 4

#include "pose_graph.hpp"

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

#include <numbers>

int main() {
    const VehicleId v1(1), v2(2), v3(3), v4(4), v5(5);

    PoseGraphBuilder slam_graph;

    // ---- Prior -------------------------------------------------------------
    const auto prior_noise =
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.3, 0.3, 0.1));
    slam_graph.addPrior(v1, VehiclePoseInWorld(gtsam::Pose2(0.0, 0.0, 0.0)), prior_noise);

    // ---- Odometry factors --------------------------------------------------
    const auto odom_noise =
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2, 0.2, 0.1));
    const double half_pi = std::numbers::pi / 2.0;

    slam_graph.addOdometry(v1, v2, RelativeVehicleTransform(gtsam::Pose2(2.0, 0.0, 0.0)),     odom_noise);
    slam_graph.addOdometry(v2, v3, RelativeVehicleTransform(gtsam::Pose2(2.0, 0.0, half_pi)), odom_noise);
    slam_graph.addOdometry(v3, v4, RelativeVehicleTransform(gtsam::Pose2(2.0, 0.0, half_pi)), odom_noise);
    slam_graph.addOdometry(v4, v5, RelativeVehicleTransform(gtsam::Pose2(2.0, 0.0, half_pi)), odom_noise);

    // ---- Loop closure ------------------------------------------------------
    const auto loop_noise =
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.5, 0.5, 0.2));
    slam_graph.addOdometry(v5, v2, RelativeVehicleTransform(gtsam::Pose2(2.0, 0.0, half_pi)), loop_noise);

    // ---- Initial estimates (deliberately noisy) ----------------------------
    slam_graph.addInitialEstimate(v1, VehiclePoseInWorld(gtsam::Pose2(0.5,  0.0,  0.2)));
    slam_graph.addInitialEstimate(v2, VehiclePoseInWorld(gtsam::Pose2(2.3,  0.1, -0.2)));
    slam_graph.addInitialEstimate(v3, VehiclePoseInWorld(gtsam::Pose2(4.1,  0.1,  half_pi)));
    slam_graph.addInitialEstimate(v4, VehiclePoseInWorld(gtsam::Pose2(3.9,  2.1,  std::numbers::pi)));
    slam_graph.addInitialEstimate(v5, VehiclePoseInWorld(gtsam::Pose2(2.1,  2.1, -half_pi)));

    // ---- Optimize ----------------------------------------------------------
    gtsam::LevenbergMarquardtOptimizer optimizer(slam_graph.graph(), slam_graph.initialValues());
    const gtsam::Values result = optimizer.optimize();

    result.print("Optimized poses:\n");

    // ---- Marginal covariances ----------------------------------------------
    gtsam::Marginals marginals(slam_graph.graph(), result);
    std::cout << "\nMarginal covariance on pose " << v1 << ":\n"
              << marginals.marginalCovariance(v1.value()) << '\n';

    return 0;
}
