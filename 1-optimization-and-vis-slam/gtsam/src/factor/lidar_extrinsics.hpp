/// @file factor/lidar_extrinsics.hpp
/// @brief Custom GTSAM factor for lidar extrinsics estimation.

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

namespace nufuse::factor {

/// @brief Factor measuring relative lidar pose while estimating body<->lidar extrinsics.
///
/// Models: Z = T_BL^{-1} * X_i^{-1} * X_j * T_BL
/// where T_BL is the body-from-lidar extrinsic calibration.
class LidarExtrinsicsFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3> {
    using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3>;

public:
    LidarExtrinsicsFactor(gtsam::Key key_Xi, gtsam::Key key_Xj, gtsam::Key key_T_BL,
                          const gtsam::Pose3& measured_Z,
                          const gtsam::SharedNoiseModel& model);

    gtsam::Vector evaluateError(
        const gtsam::Pose3& X_i, const gtsam::Pose3& X_j, const gtsam::Pose3& T_BL,
        gtsam::OptionalMatrixType H_Xi,
        gtsam::OptionalMatrixType H_Xj,
        gtsam::OptionalMatrixType H_T_BL) const override;

private:
    gtsam::Pose3 measured_Z_;
};

}  // namespace nufuse::factor

#include "factor/lidar_extrinsics.inl"
