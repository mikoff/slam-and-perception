/// @file factor/lidar_extrinsics.hpp
/// @brief Custom GTSAM factor for lidar extrinsics estimation.

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

namespace nufuse::factor {

/// @brief Factor to add radar radial velocity constraint.
///
/// Models: Z = -u^T * R_RB * R_BW * v_W
/// where R_RB is the rotation from radar to body frame,
/// R_BW is the rotation from body to world frame,
/// and v_W is the velocity in the world frame.
class RadarRadialVelocityFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Vector3> {
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Vector3>;

 private:
  const double measured_v_radial_;
  const gtsam::Vector3 u_R_;
  const gtsam::Pose3 T_BR_;

 public:
  RadarRadialVelocityFactor(gtsam::Key key_Xi, gtsam::Key key_Vi, double measured_v_radial,
                            const gtsam::Vector3& target_point_R, const gtsam::Pose3& T_BR,
                            const gtsam::SharedNoiseModel& model)
      : Base(model, key_Xi, key_Vi),
        measured_v_radial_{measured_v_radial},
        u_R_{target_point_R / target_point_R.norm()},
        T_BR_{T_BR} {}
  gtsam::Vector evaluateError(const gtsam::Pose3& X_i, const gtsam::Vector3& v_i,
                              gtsam::OptionalMatrixType H_Xi,
                              gtsam::OptionalMatrixType H_Vi) const override {
    // 1. Resolve world velocity in body frame
    gtsam::Matrix3 H_Vb_Rwb, H_Vb_Vw;
    const auto v_B = X_i.rotation().unrotate(v_i, H_Vb_Rwb, H_Vb_Vw);

    // 2. Resolve velocity in radar frame
    gtsam::Matrix3 H_Vr_Vb;
    gtsam::Vector3 v_R = T_BR_.rotation().unrotate(v_B, nullptr, H_Vr_Vb);

    // 3. Project using bearing vector
    double predicted_v_radial = -u_R_.dot(v_R);

    // 4. Compute error and Jacobians
    gtsam::Vector1 error{measured_v_radial_ - predicted_v_radial};

    // 5. Chain Rule for Jacobians
    // The derivative of a dot product (u^T * V) w.r.t V is just u^T.
    gtsam::Matrix13 H_pred_Vr = -u_R_.transpose();

    if (H_Xi) {
      const gtsam::Matrix13 H_pred_pose_rot = -1.0 * H_pred_Vr * H_Vr_Vb * H_Vb_Rwb;
      gtsam::Matrix16 H_pose_full = gtsam::Matrix16::Zero();
      H_pose_full.block<1, 3>(0, 0) = H_pred_pose_rot;
      *H_Xi = H_pose_full;
    }

    if (H_Vi) {
      *H_Vi = -1.0 * H_pred_Vr * H_Vr_Vb * H_Vb_Vw;
    }

    return error;
  }
};

}  // namespace nufuse::factor