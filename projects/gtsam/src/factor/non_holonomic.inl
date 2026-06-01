/// @file factor/non_holonomic.inl
/// @brief Inline implementation of NonHolonomicFactor.

#pragma once

namespace nufuse::factor {

inline NonHolonomicFactor::NonHolonomicFactor(gtsam::Key pose_key, gtsam::Key vel_key,
                                              const gtsam::Vector3& omega_B,
                                              const gtsam::Vector3& t_I_to_A,
                                              const gtsam::SharedNoiseModel& model)
    : Base(model, pose_key, vel_key), omega_B_(omega_B), t_I_to_A_(t_I_to_A) {}

inline gtsam::Vector NonHolonomicFactor::evaluateError(const gtsam::Pose3& pose,
                                                       const gtsam::Vector3& v_world,
                                                       gtsam::OptionalMatrixType H_pose,
                                                       gtsam::OptionalMatrixType H_vel) const {
  gtsam::Matrix3 H_R, H_v;

  // 1. Rotate world velocity into body frame
  const gtsam::Vector3 v_body = pose.rotation().unrotate(v_world, H_R, H_v);

  // 2. Apply lever arm correction: V_axle = V_B + omega_B × t_I_to_A
  const gtsam::Vector3 v_axle = v_body + omega_B_.cross(t_I_to_A_);

  // 3. Error: lateral (Y) and vertical (Z) should be zero
  //    nuScenes body frame: X=forward, Y=left, Z=up
  gtsam::Vector2 error(v_axle.y(), v_axle.z());

  // 4. Jacobians (rows 1,2 of the 3x3 unrotate Jacobians)
  if (H_pose) {
    gtsam::Matrix26 J = gtsam::Matrix26::Zero();
    J.block<2, 3>(0, 0) = H_R.bottomRows<2>();
    *H_pose = J;
  }

  if (H_vel) {
    *H_vel = H_v.bottomRows<2>();
  }

  return error;
}

}  // namespace nufuse::factor
