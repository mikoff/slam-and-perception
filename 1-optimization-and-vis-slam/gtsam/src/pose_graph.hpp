/// @file pose_graph.hpp
/// @brief Domain-typed wrapper around a GTSAM factor graph and initial values.

#pragma once

#include "domain_types.hpp"

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

/// @brief Builds a 2-D pose-graph using domain types.
///
/// All GTSAM API boundaries are encapsulated here. Callers work exclusively
/// with @c VehicleId, @c VehiclePoseInWorld, and @c RelativeVehicleTransform — raw
/// GTSAM keys and @c gtsam::Pose2 values never escape into application code.
class PoseGraphBuilder {
public:
    /// @brief Adds a prior factor on @p id with the given pose and noise model.
    void addPrior(VehicleId id,
                  const VehiclePoseInWorld& pose,
                  const gtsam::SharedNoiseModel& noise) {
        graph_.addPrior(id.value(), pose.value(), noise);
    }

    /// @brief Stores a noisy initial estimate for @p id.
    void addInitialEstimate(VehicleId id, const VehiclePoseInWorld& pose) {
        initial_.insert(id.value(), pose.value());
    }

    /// @brief Adds an odometry (or loop-closure) factor between @p from and @p to.
    void addOdometry(VehicleId from,
                     VehicleId to,
                     const RelativeVehicleTransform& meas,
                     const gtsam::SharedNoiseModel& noise) {
        graph_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(
            from.value(), to.value(), meas.value(), noise);
    }

    /// @brief Returns the accumulated factor graph (for the optimizer).
    const gtsam::NonlinearFactorGraph& graph() const { return graph_; }

    /// @brief Returns the initial-value set (for the optimizer).
    const gtsam::Values& initialValues() const { return initial_; }

private:
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_;
};
