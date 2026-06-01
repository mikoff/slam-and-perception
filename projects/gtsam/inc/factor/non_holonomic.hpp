/// @file factor/non_holonomic.hpp
/// @brief Non-holonomic constraint with lever arm compensation.
///
/// Constrains lateral (Y) and vertical (Z) velocity at the rear axle to zero
/// (nuScenes convention: X is forward, Y is left, Z is up).
/// Accounts for the lever arm between IMU and rear axle via omega × t_I_to_A.

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace nufuse::factor {

class NonHolonomicFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3> {
 private:
  using Base = gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3>;

  gtsam::Vector3 omega_B_;   // Bias-corrected angular velocity in body frame
  gtsam::Vector3 t_I_to_A_;  // Translation from IMU to rear axle in body frame

 public:
  NonHolonomicFactor(gtsam::Key pose_key, gtsam::Key vel_key, const gtsam::Vector3& omega_B,
                     const gtsam::Vector3& t_I_to_A, const gtsam::SharedNoiseModel& model);

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new NonHolonomicFactor(*this));
  }

  gtsam::Vector evaluateError(const gtsam::Pose3& pose, const gtsam::Vector3& v_world,
                              gtsam::OptionalMatrixType H_pose = nullptr,
                              gtsam::OptionalMatrixType H_vel = nullptr) const override;
};

}  // namespace nufuse::factor

#include "non_holonomic.inl"
