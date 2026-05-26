/// @file tests/test_radar_radial_velocity_factor.cpp
/// @brief Jacobian correctness tests for RadarRadialVelocityFactor using
///        GTSAM's numerical differentiation framework.

#include <cmath>
#include <gtest/gtest.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include "factor/radar_radial_velocity.hpp"

using gtsam::Matrix;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Vector1;
using gtsam::Vector3;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

namespace {

constexpr double kDelta = 1e-5;
constexpr double kTolerance = 1e-6;

/// Helper: build a factor with given parameters.
nufuse::factor::RadarRadialVelocityFactor makeFactor(
    double measured_v_radial, const Vector3& target_point_R, const Pose3& T_BR,
    const gtsam::SharedNoiseModel& noise = gtsam::noiseModel::Unit::Create(1)) {
  return nufuse::factor::RadarRadialVelocityFactor(X(0), V(0), measured_v_radial, target_point_R,
                                                   T_BR, noise);
}

/// Error-only functor for numericalDerivativeX1 (w.r.t. Pose3)
Vector1 errorWrtPose(const Pose3& X_i, const Vector3& v_i,
                     const nufuse::factor::RadarRadialVelocityFactor& factor) {
  return factor.evaluateError(X_i, v_i, nullptr, nullptr);
}

/// Error-only functor for numericalDerivativeX2 (w.r.t. velocity)
Vector1 errorWrtVel(const Pose3& X_i, const Vector3& v_i,
                    const nufuse::factor::RadarRadialVelocityFactor& factor) {
  return factor.evaluateError(X_i, v_i, nullptr, nullptr);
}

}  // namespace

// ---------------------------------------------------------------------------
// Test: Jacobian correctness at identity pose, unit velocity
// ---------------------------------------------------------------------------
TEST(RadarRadialVelocityFactor, JacobiansAtIdentity) {
  const Pose3 X_i = Pose3::Identity();
  const Vector3 v_i(1.0, 0.0, 0.0);
  const Vector3 target_point_R(10.0, 0.0, 0.0);  // directly ahead
  const Pose3 T_BR = Pose3::Identity();          // radar mounted at body origin

  auto factor = makeFactor(0.5, target_point_R, T_BR);

  // Analytical Jacobians
  Matrix H_Xi_analytical, H_Vi_analytical;
  factor.evaluateError(X_i, v_i, &H_Xi_analytical, &H_Vi_analytical);

  // Numerical Jacobians
  auto errorPose = [&](const Pose3& pose) {
    return factor.evaluateError(pose, v_i, nullptr, nullptr);
  };
  auto errorVel = [&](const Vector3& vel) {
    return factor.evaluateError(X_i, vel, nullptr, nullptr);
  };

  Matrix H_Xi_numerical = gtsam::numericalDerivative11<Vector1, Pose3>(errorPose, X_i, kDelta);
  Matrix H_Vi_numerical = gtsam::numericalDerivative11<Vector1, Vector3>(errorVel, v_i, kDelta);

  EXPECT_TRUE(H_Xi_analytical.isApprox(H_Xi_numerical, kTolerance))
      << "H_Xi mismatch:\nAnalytical:\n"
      << H_Xi_analytical << "\nNumerical:\n"
      << H_Xi_numerical;

  EXPECT_TRUE(H_Vi_analytical.isApprox(H_Vi_numerical, kTolerance))
      << "H_Vi mismatch:\nAnalytical:\n"
      << H_Vi_analytical << "\nNumerical:\n"
      << H_Vi_numerical;
}

// ---------------------------------------------------------------------------
// Test: Jacobian correctness with non-trivial rotation and offset radar
// ---------------------------------------------------------------------------
TEST(RadarRadialVelocityFactor, JacobiansWithRotatedPoseAndExtrinsics) {
  // Body rotated 45 degrees yaw
  const Pose3 X_i(Rot3::Yaw(M_PI / 4.0), gtsam::Point3(10.0, 5.0, 0.0));
  const Vector3 v_i(3.0, -2.0, 1.0);  // world-frame velocity

  // Radar mounted with a pitch offset
  const Pose3 T_BR(Rot3::Pitch(0.1), gtsam::Point3(0.5, 0.0, 0.3));

  // Target off-axis in radar frame
  const Vector3 target_point_R(5.0, 2.0, -1.0);

  auto factor = makeFactor(1.2, target_point_R, T_BR);

  Matrix H_Xi_analytical, H_Vi_analytical;
  factor.evaluateError(X_i, v_i, &H_Xi_analytical, &H_Vi_analytical);

  auto errorPose = [&](const Pose3& pose) {
    return factor.evaluateError(pose, v_i, nullptr, nullptr);
  };
  auto errorVel = [&](const Vector3& vel) {
    return factor.evaluateError(X_i, vel, nullptr, nullptr);
  };

  Matrix H_Xi_numerical = gtsam::numericalDerivative11<Vector1, Pose3>(errorPose, X_i, kDelta);
  Matrix H_Vi_numerical = gtsam::numericalDerivative11<Vector1, Vector3>(errorVel, v_i, kDelta);

  EXPECT_TRUE(H_Xi_analytical.isApprox(H_Xi_numerical, kTolerance))
      << "H_Xi mismatch:\nAnalytical:\n"
      << H_Xi_analytical << "\nNumerical:\n"
      << H_Xi_numerical;

  EXPECT_TRUE(H_Vi_analytical.isApprox(H_Vi_numerical, kTolerance))
      << "H_Vi mismatch:\nAnalytical:\n"
      << H_Vi_analytical << "\nNumerical:\n"
      << H_Vi_numerical;
}

// ---------------------------------------------------------------------------
// Test: Jacobian correctness with full 3D rotation (roll/pitch/yaw)
// ---------------------------------------------------------------------------
TEST(RadarRadialVelocityFactor, JacobiansFull3DRotation) {
  const Pose3 X_i(Rot3::RzRyRx(0.3, -0.2, 1.1), gtsam::Point3(-5.0, 3.0, 2.0));
  const Vector3 v_i(-1.5, 4.0, 0.7);

  // Significant radar extrinsic rotation
  const Pose3 T_BR(Rot3::RzRyRx(0.05, -0.1, 0.15), gtsam::Point3(1.0, -0.5, 0.2));

  const Vector3 target_point_R(3.0, -1.0, 2.0);

  auto factor = makeFactor(-0.8, target_point_R, T_BR);

  Matrix H_Xi_analytical, H_Vi_analytical;
  factor.evaluateError(X_i, v_i, &H_Xi_analytical, &H_Vi_analytical);

  auto errorPose = [&](const Pose3& pose) {
    return factor.evaluateError(pose, v_i, nullptr, nullptr);
  };
  auto errorVel = [&](const Vector3& vel) {
    return factor.evaluateError(X_i, vel, nullptr, nullptr);
  };

  Matrix H_Xi_numerical = gtsam::numericalDerivative11<Vector1, Pose3>(errorPose, X_i, kDelta);
  Matrix H_Vi_numerical = gtsam::numericalDerivative11<Vector1, Vector3>(errorVel, v_i, kDelta);

  EXPECT_TRUE(H_Xi_analytical.isApprox(H_Xi_numerical, kTolerance))
      << "H_Xi mismatch:\nAnalytical:\n"
      << H_Xi_analytical << "\nNumerical:\n"
      << H_Xi_numerical;

  EXPECT_TRUE(H_Vi_analytical.isApprox(H_Vi_numerical, kTolerance))
      << "H_Vi mismatch:\nAnalytical:\n"
      << H_Vi_analytical << "\nNumerical:\n"
      << H_Vi_numerical;
}

// ---------------------------------------------------------------------------
// Test: Factor produces zero error at perfect measurement
// ---------------------------------------------------------------------------
TEST(RadarRadialVelocityFactor, ZeroErrorAtPerfectMeasurement) {
  const Pose3 X_i(Rot3::Yaw(0.5), gtsam::Point3(1.0, 2.0, 0.0));
  const Vector3 v_i(2.0, 1.0, 0.0);
  const Pose3 T_BR = Pose3::Identity();
  const Vector3 target_point_R(1.0, 0.0, 0.0);  // pure +x direction in radar

  // Compute what the "true" radial velocity should be
  // v_B = R_BW * v_W, v_R = R_RB * v_B, predicted = -u^T * v_R
  const Vector3 v_B = X_i.rotation().unrotate(v_i);
  const Vector3 v_R = T_BR.rotation().unrotate(v_B);
  const Vector3 u_R = target_point_R.normalized();
  const double true_radial = -u_R.dot(v_R);

  auto factor = makeFactor(true_radial, target_point_R, T_BR);

  Vector1 error = factor.evaluateError(X_i, v_i, nullptr, nullptr);
  EXPECT_NEAR(error(0), 0.0, 1e-12);
}

// ---------------------------------------------------------------------------
// Test: Full Jacobian test using GTSAM's linearizeNumerically (Values-based)
// ---------------------------------------------------------------------------
TEST(RadarRadialVelocityFactor, FactorTestingLinearize) {
  const Pose3 X_i(Rot3::RzRyRx(0.1, 0.2, -0.3), gtsam::Point3(1.0, -2.0, 3.0));
  const Vector3 v_i(1.5, -0.5, 2.0);
  const Pose3 T_BR(Rot3::Pitch(0.05), gtsam::Point3(0.3, 0.0, 0.1));
  const Vector3 target_point_R(8.0, 1.0, -0.5);

  auto factor = makeFactor(0.7, target_point_R, T_BR);

  gtsam::Values values;
  values.insert(X(0), X_i);
  values.insert(V(0), v_i);

  // Use GTSAM's internal testFactorJacobians which compares linearize() output
  // against numerically computed Jacobians.
  EXPECT_TRUE(gtsam::internal::testFactorJacobians("RadarRadialVelocityFactor", factor, values,
                                                   kDelta, kTolerance));
}

// ---------------------------------------------------------------------------
// Test: Multiple random-ish linearization points
// ---------------------------------------------------------------------------
TEST(RadarRadialVelocityFactor, JacobiansMultipleLinPoints) {
  const Pose3 T_BR(Rot3::RzRyRx(0.02, -0.03, 0.01), gtsam::Point3(0.4, -0.1, 0.15));
  const Vector3 target_point_R(6.0, 3.0, 1.0);

  struct TestCase {
    Pose3 pose;
    Vector3 vel;
    double meas;
  };

  std::vector<TestCase> cases = {
      {Pose3(Rot3::Yaw(0.0), gtsam::Point3(0, 0, 0)), Vector3(1, 0, 0), 0.5},
      {Pose3(Rot3::Yaw(M_PI), gtsam::Point3(5, 5, 0)), Vector3(-2, 3, 1), -1.0},
      {Pose3(Rot3::RzRyRx(1.0, -0.5, 0.7), gtsam::Point3(-3, 8, -1)), Vector3(0.1, -4.0, 2.5), 2.3},
      {Pose3(Rot3::RzRyRx(-0.8, 0.4, 2.1), gtsam::Point3(20, -10, 5)), Vector3(5, 5, 5), -3.1},
  };

  for (size_t i = 0; i < cases.size(); ++i) {
    const auto& tc = cases[i];
    auto factor = makeFactor(tc.meas, target_point_R, T_BR);

    Matrix H_Xi_analytical, H_Vi_analytical;
    factor.evaluateError(tc.pose, tc.vel, &H_Xi_analytical, &H_Vi_analytical);

    auto errorPose = [&](const Pose3& pose) {
      return factor.evaluateError(pose, tc.vel, nullptr, nullptr);
    };
    auto errorVel = [&](const Vector3& vel) {
      return factor.evaluateError(tc.pose, vel, nullptr, nullptr);
    };

    Matrix H_Xi_num = gtsam::numericalDerivative11<Vector1, Pose3>(errorPose, tc.pose, kDelta);
    Matrix H_Vi_num = gtsam::numericalDerivative11<Vector1, Vector3>(errorVel, tc.vel, kDelta);

    EXPECT_TRUE(H_Xi_analytical.isApprox(H_Xi_num, kTolerance))
        << "Case " << i << " H_Xi mismatch:\nAnalytical:\n"
        << H_Xi_analytical << "\nNumerical:\n"
        << H_Xi_num;

    EXPECT_TRUE(H_Vi_analytical.isApprox(H_Vi_num, kTolerance))
        << "Case " << i << " H_Vi mismatch:\nAnalytical:\n"
        << H_Vi_analytical << "\nNumerical:\n"
        << H_Vi_num;
  }
}

// ---------------------------------------------------------------------------
// Test: Jacobian w.r.t velocity is linear (constant Jacobian)
// ---------------------------------------------------------------------------
TEST(RadarRadialVelocityFactor, VelocityJacobianIsConstant) {
  // The error is linear in v_i (since the rotation is fixed for a given pose),
  // so the Jacobian w.r.t. v_i should be the same regardless of v_i value.
  const Pose3 X_i(Rot3::RzRyRx(0.5, -0.3, 1.0), gtsam::Point3(1, 2, 3));
  const Pose3 T_BR(Rot3::Yaw(0.1), gtsam::Point3(0.2, 0, 0.1));
  const Vector3 target_point_R(4.0, 1.0, 0.5);

  auto factor = makeFactor(0.0, target_point_R, T_BR);

  Matrix H_Vi_a, H_Vi_b;
  factor.evaluateError(X_i, Vector3(1, 2, 3), nullptr, &H_Vi_a);
  factor.evaluateError(X_i, Vector3(-5, 10, -7), nullptr, &H_Vi_b);

  EXPECT_TRUE(H_Vi_a.isApprox(H_Vi_b, 1e-12))
      << "Velocity Jacobian should be constant w.r.t. velocity value.\n"
      << "At v=(1,2,3):\n"
      << H_Vi_a << "\nAt v=(-5,10,-7):\n"
      << H_Vi_b;
}
