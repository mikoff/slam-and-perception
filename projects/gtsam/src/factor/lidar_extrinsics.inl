/// @file factor/lidar_extrinsics.inl
/// @brief Implementation of LidarExtrinsicsFactor.

#pragma once

namespace nufuse::factor {

inline LidarExtrinsicsFactor::LidarExtrinsicsFactor(
    gtsam::Key key_Xi, gtsam::Key key_Xj, gtsam::Key key_T_BL,
    const gtsam::Pose3& measured_Z, const gtsam::SharedNoiseModel& model)
    : Base(model, key_Xi, key_Xj, key_T_BL)
    , measured_Z_(measured_Z) {}

inline gtsam::Vector LidarExtrinsicsFactor::evaluateError(
    const gtsam::Pose3& X_i, const gtsam::Pose3& X_j, const gtsam::Pose3& T_BL,
    gtsam::OptionalMatrixType H_Xi,
    gtsam::OptionalMatrixType H_Xj,
    gtsam::OptionalMatrixType H_T_BL) const {

    using gtsam::Matrix6;
    using gtsam::Pose3;

    Matrix6 H_Pi_Xi, H_Pi_TBL;
    const Pose3 P_i = X_i.compose(T_BL,
                                  H_Xi ? &H_Pi_Xi : nullptr,
                                  H_T_BL ? &H_Pi_TBL : nullptr);

    Matrix6 H_Pj_Xj, H_Pj_TBL;
    const Pose3 P_j = X_j.compose(T_BL,
                                  H_Xj ? &H_Pj_Xj : nullptr,
                                  H_T_BL ? &H_Pj_TBL : nullptr);

    const bool need_Pi = H_Xi || H_T_BL;
    const bool need_Pj = H_Xj || H_T_BL;

    Matrix6 H_pred_Pi, H_pred_Pj;
    const Pose3 predicted_Z = P_i.between(P_j,
                                          need_Pi ? &H_pred_Pi : nullptr,
                                          need_Pj ? &H_pred_Pj : nullptr);

    Matrix6 H_err_pred;
    const gtsam::Vector6 error = measured_Z_.localCoordinates(
        predicted_Z, nullptr, (need_Pi || need_Pj) ? &H_err_pred : nullptr);

    if (H_Xi)   { *H_Xi = H_err_pred * H_pred_Pi * H_Pi_Xi; }
    if (H_Xj)   { *H_Xj = H_err_pred * H_pred_Pj * H_Pj_Xj; }
    if (H_T_BL) { *H_T_BL = H_err_pred * (H_pred_Pi * H_Pi_TBL + H_pred_Pj * H_Pj_TBL); }

    return error;
}

}  // namespace nufuse::factor
