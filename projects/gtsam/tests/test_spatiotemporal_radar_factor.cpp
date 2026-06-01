/// @file tests/test_spatiotemporal_radar_factor.cpp
/// @brief Jacobian correctness tests for SpatiotemporalRadarFactor.

#include <cmath>
#include <gtest/gtest.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include "factor/spatiotemporal_radar.hpp"

using gtsam::Matrix;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Vector1;
using gtsam::Vector3;
using gtsam::symbol_shorthand::R;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

namespace {

constexpr double kDelta = 1e-5;
constexpr double kTolerance = 1e-6;

nufuse::factor::SpatiotemporalRadarFactor makeFactor(
    double measured, const Vector3& target_R, const Vector3& omega_B,
    const gtsam::SharedNoiseModel& noise = gtsam::noiseModel::Unit::Create(1)) {
  return nufuse::factor::SpatiotemporalRadarFactor(X(0), V(0), R(0), measured, target_R, omega_B,
                                                   noise);
}

}  // namespace

// ---------------------------------------------------------------------------
// Test: Jacobians at identity, zero angular velocity (no lever-arm)
// ---------------------------------------------------------------------------
TEST(SpatiotemporalRadarFactor, JacobiansAtIdentityNoLeverArm) {
  const Pose3 X_i = Pose3::Identity();
  const Vector3 v_i(1.0, 0.0, 0.0);
  const Pose3 T_BR = Pose3::Identity();
  const Vector3 omega_B = Vector3::Zero();
  const Vector3 target_R(10.0, 0.0, 0.0);

  auto factor = makeFactor(0.5, target_R, omega_B);

  gtsam::Values values;
  values.insert(X(0), X_i);
  values.insert(V(0), v_i);
  values.insert(R(0), T_BR);

  EXPECT_TRUE(
      gtsam::internal::testFactorJacobians("Identity_NoLever", factor, values, kDelta, kTolerance));
}

// ---------------------------------------------------------------------------
// Test: Jacobians with non-zero omega (lever-arm active)
// ---------------------------------------------------------------------------
TEST(SpatiotemporalRadarFactor, JacobiansWithLeverArm) {
  const Pose3 X_i(Rot3::Yaw(M_PI / 4.0), gtsam::Point3(10.0, 5.0, 0.0));
  const Vector3 v_i(3.0, -2.0, 1.0);
  const Pose3 T_BR(Rot3::Pitch(0.1), gtsam::Point3(0.5, 0.0, 0.3));
  const Vector3 omega_B(0.0, 0.0, 0.5);  // yaw rate
  const Vector3 target_R(5.0, 2.0, -1.0);

  auto factor = makeFactor(1.2, target_R, omega_B);

  gtsam::Values values;
  values.insert(X(0), X_i);
  values.insert(V(0), v_i);
  values.insert(R(0), T_BR);

  EXPECT_TRUE(gtsam::internal::testFactorJacobians("LeverArm", factor, values, kDelta, kTolerance));
}

// ---------------------------------------------------------------------------
// Test: Full 3D rotation and significant angular velocity
// ---------------------------------------------------------------------------
TEST(SpatiotemporalRadarFactor, JacobiansFull3D) {
  const Pose3 X_i(Rot3::RzRyRx(0.3, -0.2, 1.1), gtsam::Point3(-5.0, 3.0, 2.0));
  const Vector3 v_i(-1.5, 4.0, 0.7);
  const Pose3 T_BR(Rot3::RzRyRx(0.05, -0.1, 0.15), gtsam::Point3(1.0, -0.5, 0.2));
  const Vector3 omega_B(0.1, -0.3, 0.8);
  const Vector3 target_R(3.0, -1.0, 2.0);

  auto factor = makeFactor(-0.8, target_R, omega_B);

  gtsam::Values values;
  values.insert(X(0), X_i);
  values.insert(V(0), v_i);
  values.insert(R(0), T_BR);

  EXPECT_TRUE(gtsam::internal::testFactorJacobians("Full3D", factor, values, kDelta, kTolerance));
}

// ---------------------------------------------------------------------------
// Test: Zero error at perfect measurement (with lever-arm)
// ---------------------------------------------------------------------------
TEST(SpatiotemporalRadarFactor, ZeroErrorAtPerfectMeasurement) {
  const Pose3 X_i(Rot3::Yaw(0.5), gtsam::Point3(1.0, 2.0, 0.0));
  const Vector3 v_i(2.0, 1.0, 0.0);
  const Pose3 T_BR(Rot3::Yaw(0.1), gtsam::Point3(0.3, 0.0, 0.1));
  const Vector3 omega_B(0.0, 0.0, 1.0);
  const Vector3 target_R(1.0, 0.0, 0.0);

  // Compute ground-truth radial velocity
  const Vector3 v_B = X_i.rotation().unrotate(v_i);
  const Vector3 lever = omega_B.cross(T_BR.translation());
  const Vector3 v_corrected = v_B + lever;
  const Vector3 v_R = T_BR.rotation().unrotate(v_corrected);
  const Vector3 u_R = target_R.normalized();
  const double true_radial = -u_R.dot(v_R);

  auto factor = makeFactor(true_radial, target_R, omega_B);
  const Vector1 error = factor.evaluateError(X_i, v_i, T_BR, nullptr, nullptr, nullptr);
  EXPECT_NEAR(error(0), 0.0, 1e-12);
}

// ---------------------------------------------------------------------------
// Test: Lever-arm actually contributes (non-zero omega changes the result)
// ---------------------------------------------------------------------------
TEST(SpatiotemporalRadarFactor, LeverArmChangesResult) {
  const Pose3 X_i(Rot3::Yaw(0.3), gtsam::Point3(0, 0, 0));
  const Vector3 v_i(2.0, 0.0, 0.0);
  const Pose3 T_BR(Rot3::Identity(), gtsam::Point3(1.0, 0.0, 0.0));  // radar 1m forward
  const Vector3 target_R(1.0, 1.0, 0.0);  // off-axis so lever-arm projects non-zero

  // With zero omega
  auto factor_no_lever = makeFactor(0.0, target_R, Vector3::Zero());
  const Vector1 err_no = factor_no_lever.evaluateError(X_i, v_i, T_BR, nullptr, nullptr, nullptr);

  // With non-zero omega (yaw rate)
  const Vector3 omega_B(0.0, 0.0, 1.0);
  auto factor_lever = makeFactor(0.0, target_R, omega_B);
  const Vector1 err_with = factor_lever.evaluateError(X_i, v_i, T_BR, nullptr, nullptr, nullptr);

  // They must differ (lever arm contribution is non-zero for this config)
  EXPECT_GT(std::abs(err_no(0) - err_with(0)), 1e-6);
}

// ---------------------------------------------------------------------------
// Test: Multiple random linearization points
// ---------------------------------------------------------------------------
TEST(SpatiotemporalRadarFactor, JacobiansMultipleLinPoints) {
  struct TestCase {
    Pose3 pose;
    Vector3 vel;
    Pose3 T_BR;
    Vector3 omega;
    double meas;
  };

  const Vector3 target_R(6.0, 3.0, 1.0);
  std::vector<TestCase> cases = {
      {Pose3(Rot3::Yaw(0.0), gtsam::Point3(0, 0, 0)), Vector3(1, 0, 0),
       Pose3(Rot3::Identity(), gtsam::Point3(0.5, 0, 0)), Vector3(0, 0, 0.2), 0.5},
      {Pose3(Rot3::Yaw(M_PI), gtsam::Point3(5, 5, 0)), Vector3(-2, 3, 1),
       Pose3(Rot3::Yaw(0.3), gtsam::Point3(1.0, -0.2, 0.1)), Vector3(0.1, -0.1, 0.5), -1.0},
      {Pose3(Rot3::RzRyRx(1.0, -0.5, 0.7), gtsam::Point3(-3, 8, -1)), Vector3(0.1, -4.0, 2.5),
       Pose3(Rot3::RzRyRx(0.02, -0.05, 0.1), gtsam::Point3(0.8, 0.3, 0.15)),
       Vector3(-0.2, 0.4, 1.2), 2.3},
      {Pose3(Rot3::RzRyRx(-0.8, 0.4, 2.1), gtsam::Point3(20, -10, 5)), Vector3(5, 5, 5),
       Pose3(Rot3::RzRyRx(0.1, 0.1, -0.2), gtsam::Point3(0.4, -0.1, 0.2)), Vector3(0.3, -0.5, 0.7),
       -3.1},
  };

  for (size_t i = 0; i < cases.size(); ++i) {
    const auto& tc = cases[i];
    auto factor = makeFactor(tc.meas, target_R, tc.omega);

    gtsam::Values values;
    values.insert(X(0), tc.pose);
    values.insert(V(0), tc.vel);
    values.insert(R(0), tc.T_BR);

    EXPECT_TRUE(gtsam::internal::testFactorJacobians("MultiPoint_" + std::to_string(i), factor,
                                                     values, kDelta, kTolerance))
        << "Failed at case " << i;
  }
}

// ---------------------------------------------------------------------------
// Test: Extrinsic prior pattern (demonstrates observable DOF locking)
// ---------------------------------------------------------------------------
TEST(SpatiotemporalRadarFactor, ExtrinsicPriorNoiseModel) {
  // Verify the prior noise model can be constructed as documented
  auto extrinsic_prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 1e-5, 1e-5, 1.0,  // Roll(locked), Pitch(locked), Yaw(free)
       1.0, 1.0, 1e-5                        // X(free), Y(free), Z(locked)
       )
          .finished());

  ASSERT_NE(extrinsic_prior_noise, nullptr);
  EXPECT_DOUBLE_EQ(extrinsic_prior_noise->sigmas()(0), 1e-5);
  EXPECT_DOUBLE_EQ(extrinsic_prior_noise->sigmas()(2), 1.0);
  EXPECT_DOUBLE_EQ(extrinsic_prior_noise->sigmas()(5), 1e-5);
}
