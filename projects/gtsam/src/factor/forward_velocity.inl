/// @file factor/forward_velocity.inl
/// @brief Inline implementation of ForwardVelocityScaleFactor.

#pragma once

namespace nufuse::factor {

inline ForwardVelocityScaleFactor::ForwardVelocityScaleFactor(
    gtsam::Key pose_i, gtsam::Key pose_j, gtsam::Key scale_key, double measured_v, double dt,
    const gtsam::Vector3& omega_B, const gtsam::Vector3& lever_arm,
    const gtsam::SharedNoiseModel& model)
    : Base(model, pose_i, pose_j, scale_key),
      measured_v_(measured_v),
      dt_(dt),
      omega_B_(omega_B),
      lever_arm_(lever_arm) {}

inline gtsam::Vector ForwardVelocityScaleFactor::evaluateError(
    const gtsam::Pose3& p_i, const gtsam::Pose3& p_j, const double& scale_delta,
    gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2,
    gtsam::OptionalMatrixType H3) const {
  // 1. Relative pose: T_ij = p_i^{-1} * p_j
  gtsam::Matrix6 H_between_i, H_between_j;
  const gtsam::Pose3 T_ij =
      p_i.between(p_j, H1 ? &H_between_i : nullptr, H2 ? &H_between_j : nullptr);

  // 2. Extract relative translation
  gtsam::Matrix36 H_t_Tij;
  const gtsam::Point3 t_ij = T_ij.translation((H1 || H2) ? &H_t_Tij : nullptr);

  // 3. Lever arm correction: velocity induced at odom frame by rotation
  const gtsam::Vector3 cross_product = omega_B_.cross(lever_arm_);
  const double lever_correction = cross_product.x();  // X = forward axis

  // 4. Estimated forward velocity from poses
  const double v_est = t_ij.x() / dt_;

  // 5. Expected velocity: (1 + delta) * measured - lever_correction
  const double v_exp = (1.0 + scale_delta) * measured_v_ - lever_correction;

  // 6. Error
  const double error = v_est - v_exp;

  // 7. Jacobians
  if (H1 || H2) {
    // d(error)/d(t_ij.x) = 1/dt, chain through translation and between
    gtsam::Matrix13 J_axis = gtsam::Matrix13::Zero();
    J_axis(0, 0) = 1.0 / dt_;  // X-axis (forward)
    const gtsam::Matrix16 J_combined = J_axis * H_t_Tij;

    if (H1) *H1 = J_combined * H_between_i;
    if (H2) *H2 = J_combined * H_between_j;
  }

  if (H3) {
    // d(error)/d(delta) = -measured_v_
    *H3 = (gtsam::Matrix11() << -measured_v_).finished();
  }

  return (gtsam::Vector1() << error).finished();
}

}  // namespace nufuse::factor
