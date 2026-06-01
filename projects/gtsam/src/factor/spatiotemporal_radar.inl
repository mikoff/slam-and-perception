/// @file factor/spatiotemporal_radar.inl
/// @brief Implementation of SpatiotemporalRadarFactor.

#pragma once

#include <gtsam/base/Matrix.h>

namespace nufuse::factor {

inline SpatiotemporalRadarFactor::SpatiotemporalRadarFactor(gtsam::Key pose_key, gtsam::Key vel_key,
                                                            gtsam::Key extrinsic_key,
                                                            double measured_v_radial,
                                                            const gtsam::Vector3& unit_bearing_vector,
                                                            const gtsam::Vector3& omega_B,
                                                            const gtsam::SharedNoiseModel& model)
    : Base(model, pose_key, vel_key, extrinsic_key),
      measured_v_radial_{measured_v_radial},
      u_R_{unit_bearing_vector.normalized()},
      omega_B_{omega_B} {}

inline gtsam::Vector SpatiotemporalRadarFactor::evaluateError(
    const gtsam::Pose3& X_i, const gtsam::Vector3& v_i, const gtsam::Pose3& T_BR,
    gtsam::OptionalMatrixType H_Xi, gtsam::OptionalMatrixType H_Vi,
    gtsam::OptionalMatrixType H_T_BR) const {
  // 1. Body-frame velocity: v_B = R_WB^T * v_W
  gtsam::Matrix3 H_vB_rot, H_vB_vW;
  const gtsam::Vector3 v_B = X_i.rotation().unrotate(v_i, H_Xi ? &H_vB_rot : nullptr, &H_vB_vW);

  // 2. Lever-arm: v_corrected = v_B + omega_B × t_BR
  const gtsam::Vector3 t_BR = T_BR.translation();
  const gtsam::Vector3 v_corrected = v_B + omega_B_.cross(t_BR);

  // 3. Radar-frame velocity: v_R = R_BR^T * v_corrected
  gtsam::Matrix3 H_vR_rot, H_vR_vc;
  const gtsam::Vector3 v_R =
      T_BR.rotation().unrotate(v_corrected, H_T_BR ? &H_vR_rot : nullptr, &H_vR_vc);

  // 4. Radial projection and error
  const double predicted = -u_R_.dot(v_R);
  const gtsam::Matrix13 H_pred_vR = -u_R_.transpose();

  // 5. Jacobians: error = measured - predicted, so ∂error/∂x = -∂predicted/∂x
  if (H_Xi) {
    const gtsam::Matrix13 H_rot = -1.0 * H_pred_vR * H_vR_vc * H_vB_rot;
    *H_Xi = (gtsam::Matrix16() << H_rot, gtsam::Matrix13::Zero()).finished();
  }

  if (H_Vi) {
    *H_Vi = -1.0 * H_pred_vR * H_vR_vc * H_vB_vW;
  }

  if (H_T_BR) {
    // Rotation part: directly from unrotate Jacobian
    // Translation part: ∂v_R/∂δρ = R_BR^T * [ω_B]× * R_BR (Pose3 right perturbation)
    const gtsam::Matrix3 R_BR = T_BR.rotation().matrix();
    const gtsam::Matrix3 omega_skew = gtsam::skewSymmetric(omega_B_(0), omega_B_(1), omega_B_(2));
    const gtsam::Matrix3 H_vR_rho = H_vR_vc * omega_skew * R_BR;

    gtsam::Matrix16 J;
    J.block<1, 3>(0, 0) = -1.0 * H_pred_vR * H_vR_rot;
    J.block<1, 3>(0, 3) = -1.0 * H_pred_vR * H_vR_rho;
    *H_T_BR = J;
  }

  return gtsam::Vector1{measured_v_radial_ - predicted};
}

}  // namespace nufuse::factor
