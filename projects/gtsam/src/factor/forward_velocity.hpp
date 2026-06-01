/// @file factor/forward_velocity.hpp
/// @brief Forward velocity scale factor.
///
/// Constrains the forward-axis velocity derived from consecutive poses to match
/// a measured wheel/odom speed multiplied by an estimated scale correction.
/// nuScenes convention: X is forward.

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace nufuse::factor {

class ForwardVelocityScaleFactor
    : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose3, double> {
 private:
  using Base = gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose3, double>;

  double measured_v_;         // Forward speed from odometry/CAN [m/s]
  double dt_;                 // Time between poses [s]
  gtsam::Vector3 omega_B_;    // Angular velocity in body frame [rad/s]
  gtsam::Vector3 lever_arm_;  // Translation from IMU to odom reference point [m]

 public:
  ForwardVelocityScaleFactor(gtsam::Key pose_i, gtsam::Key pose_j, gtsam::Key scale_key,
                             double measured_v, double dt, const gtsam::Vector3& omega_B,
                             const gtsam::Vector3& lever_arm, const gtsam::SharedNoiseModel& model);

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new ForwardVelocityScaleFactor(*this));
  }

  gtsam::Vector evaluateError(const gtsam::Pose3& p_i, const gtsam::Pose3& p_j,
                              const double& scale_delta, gtsam::OptionalMatrixType H1 = nullptr,
                              gtsam::OptionalMatrixType H2 = nullptr,
                              gtsam::OptionalMatrixType H3 = nullptr) const override;
};

}  // namespace nufuse::factor

#include "forward_velocity.inl"
