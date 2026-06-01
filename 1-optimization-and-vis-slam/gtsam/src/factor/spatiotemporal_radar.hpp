/// @file factor/spatiotemporal_radar.hpp
/// @brief 3-way radar factor estimating body pose, velocity, and radar extrinsics.
///
/// Models: v_radial = -u_R^T * R_RB * (R_BW * v_W + omega_B × t_BR)
/// Lever-arm effect included: V_sensor = V_B + omega_B × t_BR

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

namespace nufuse::factor {

class SpatiotemporalRadarFactor
    : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Vector3, gtsam::Pose3> {
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Vector3, gtsam::Pose3>;

 private:
  double measured_v_radial_;
  gtsam::Vector3 u_R_;      // unit bearing in radar frame
  gtsam::Vector3 omega_B_;  // angular velocity in body frame (from IMU)

 public:
  SpatiotemporalRadarFactor(gtsam::Key pose_key, gtsam::Key vel_key, gtsam::Key extrinsic_key,
                            double measured_v_radial, const gtsam::Vector3& unit_bearing_vector,
                            const gtsam::Vector3& omega_B, const gtsam::SharedNoiseModel& model);

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new SpatiotemporalRadarFactor(*this));
  }

  gtsam::Vector evaluateError(const gtsam::Pose3& X_i, const gtsam::Vector3& v_i,
                              const gtsam::Pose3& T_BR, gtsam::OptionalMatrixType H_Xi,
                              gtsam::OptionalMatrixType H_Vi,
                              gtsam::OptionalMatrixType H_T_BR) const override;
};

}  // namespace nufuse::factor

#include "factor/spatiotemporal_radar.inl"
