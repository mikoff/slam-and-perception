/// @file tests/test_lidar_extrinsics_factor.cpp
/// @brief Jacobian correctness tests for LidarExtrinsicsFactor using
///        GTSAM's numerical differentiation framework.

#include <cmath>
#include <gtest/gtest.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include "factor/lidar_extrinsics.hpp"

using gtsam::Matrix;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Vector6;
using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::X;

namespace {

constexpr double kDelta = 1e-5;
constexpr double kTolerance = 1e-6;

/// Helper: build a factor with given parameters.
nufuse::factor::LidarExtrinsicsFactor makeFactor(
    const Pose3& measured_Z,
    const gtsam::SharedNoiseModel& noise = gtsam::noiseModel::Unit::Create(6)) {
  return nufuse::factor::LidarExtrinsicsFactor(X(0), X(1), L(0), measured_Z, noise);
}

}  // namespace

// ---------------------------------------------------------------------------
// Test: Jacobian correctness at identity poses
// ---------------------------------------------------------------------------
TEST(LidarExtrinsicsFactor, JacobiansAtIdentity) {
  const Pose3 X_i = Pose3::Identity();
  const Pose3 X_j = Pose3(Rot3::Yaw(0.1), gtsam::Point3(1.0, 0.0, 0.0));
  const Pose3 T_BL = Pose3::Identity();

  // Compute ground truth measurement: T_BL^-1 * X_i^-1 * X_j * T_BL
  const Pose3 measured_Z = T_BL.inverse() * X_i.inverse() * X_j * T_BL;

  auto factor = makeFactor(measured_Z);

  Matrix H_Xi, H_Xj, H_TBL;
  factor.evaluateError(X_i, X_j, T_BL, &H_Xi, &H_Xj, &H_TBL);

  auto errorFn = [&](const Pose3& xi, const Pose3& xj, const Pose3& tbl) {
    return factor.evaluateError(xi, xj, tbl, nullptr, nullptr, nullptr);
  };

  Matrix H_Xi_num =
      gtsam::numericalDerivative31<Vector6, Pose3, Pose3, Pose3>(errorFn, X_i, X_j, T_BL, kDelta);
  Matrix H_Xj_num =
      gtsam::numericalDerivative32<Vector6, Pose3, Pose3, Pose3>(errorFn, X_i, X_j, T_BL, kDelta);
  Matrix H_TBL_num =
      gtsam::numericalDerivative33<Vector6, Pose3, Pose3, Pose3>(errorFn, X_i, X_j, T_BL, kDelta);

  EXPECT_TRUE(H_Xi.isApprox(H_Xi_num, kTolerance)) << "H_Xi mismatch:\nAnalytical:\n"
                                                   << H_Xi << "\nNumerical:\n"
                                                   << H_Xi_num;
  EXPECT_TRUE(H_Xj.isApprox(H_Xj_num, kTolerance)) << "H_Xj mismatch:\nAnalytical:\n"
                                                   << H_Xj << "\nNumerical:\n"
                                                   << H_Xj_num;
  EXPECT_TRUE(H_TBL.isApprox(H_TBL_num, kTolerance)) << "H_TBL mismatch:\nAnalytical:\n"
                                                     << H_TBL << "\nNumerical:\n"
                                                     << H_TBL_num;
}

// ---------------------------------------------------------------------------
// Test: Jacobian correctness with non-trivial extrinsics
// ---------------------------------------------------------------------------
TEST(LidarExtrinsicsFactor, JacobiansWithNonTrivialExtrinsics) {
  const Pose3 X_i(Rot3::Yaw(0.3), gtsam::Point3(1.0, 2.0, 0.0));
  const Pose3 X_j(Rot3::Yaw(0.5), gtsam::Point3(3.0, 2.5, 0.0));
  // Lidar mounted with offset and rotation
  const Pose3 T_BL(Rot3::RzRyRx(0.01, -0.02, 0.03), gtsam::Point3(0.5, 0.0, 1.5));

  const Pose3 measured_Z = T_BL.inverse() * X_i.inverse() * X_j * T_BL;
  auto factor = makeFactor(measured_Z);

  Matrix H_Xi, H_Xj, H_TBL;
  factor.evaluateError(X_i, X_j, T_BL, &H_Xi, &H_Xj, &H_TBL);

  auto errorFn = [&](const Pose3& xi, const Pose3& xj, const Pose3& tbl) {
    return factor.evaluateError(xi, xj, tbl, nullptr, nullptr, nullptr);
  };

  Matrix H_Xi_num =
      gtsam::numericalDerivative31<Vector6, Pose3, Pose3, Pose3>(errorFn, X_i, X_j, T_BL, kDelta);
  Matrix H_Xj_num =
      gtsam::numericalDerivative32<Vector6, Pose3, Pose3, Pose3>(errorFn, X_i, X_j, T_BL, kDelta);
  Matrix H_TBL_num =
      gtsam::numericalDerivative33<Vector6, Pose3, Pose3, Pose3>(errorFn, X_i, X_j, T_BL, kDelta);

  EXPECT_TRUE(H_Xi.isApprox(H_Xi_num, kTolerance)) << "H_Xi mismatch:\nAnalytical:\n"
                                                   << H_Xi << "\nNumerical:\n"
                                                   << H_Xi_num;
  EXPECT_TRUE(H_Xj.isApprox(H_Xj_num, kTolerance)) << "H_Xj mismatch:\nAnalytical:\n"
                                                   << H_Xj << "\nNumerical:\n"
                                                   << H_Xj_num;
  EXPECT_TRUE(H_TBL.isApprox(H_TBL_num, kTolerance)) << "H_TBL mismatch:\nAnalytical:\n"
                                                     << H_TBL << "\nNumerical:\n"
                                                     << H_TBL_num;
}

// ---------------------------------------------------------------------------
// Test: Jacobian correctness with full 3D rotation
// ---------------------------------------------------------------------------
TEST(LidarExtrinsicsFactor, JacobiansFull3DPoses) {
  const Pose3 X_i(Rot3::RzRyRx(0.5, -0.3, 1.2), gtsam::Point3(-2.0, 5.0, 1.0));
  const Pose3 X_j(Rot3::RzRyRx(-0.2, 0.8, -0.5), gtsam::Point3(4.0, -1.0, 3.0));
  const Pose3 T_BL(Rot3::RzRyRx(0.1, -0.05, 0.2), gtsam::Point3(0.3, -0.2, 1.8));

  const Pose3 measured_Z = T_BL.inverse() * X_i.inverse() * X_j * T_BL;
  auto factor = makeFactor(measured_Z);

  Matrix H_Xi, H_Xj, H_TBL;
  factor.evaluateError(X_i, X_j, T_BL, &H_Xi, &H_Xj, &H_TBL);

  auto errorFn = [&](const Pose3& xi, const Pose3& xj, const Pose3& tbl) {
    return factor.evaluateError(xi, xj, tbl, nullptr, nullptr, nullptr);
  };

  Matrix H_Xi_num =
      gtsam::numericalDerivative31<Vector6, Pose3, Pose3, Pose3>(errorFn, X_i, X_j, T_BL, kDelta);
  Matrix H_Xj_num =
      gtsam::numericalDerivative32<Vector6, Pose3, Pose3, Pose3>(errorFn, X_i, X_j, T_BL, kDelta);
  Matrix H_TBL_num =
      gtsam::numericalDerivative33<Vector6, Pose3, Pose3, Pose3>(errorFn, X_i, X_j, T_BL, kDelta);

  EXPECT_TRUE(H_Xi.isApprox(H_Xi_num, kTolerance)) << "H_Xi mismatch:\nAnalytical:\n"
                                                   << H_Xi << "\nNumerical:\n"
                                                   << H_Xi_num;
  EXPECT_TRUE(H_Xj.isApprox(H_Xj_num, kTolerance)) << "H_Xj mismatch:\nAnalytical:\n"
                                                   << H_Xj << "\nNumerical:\n"
                                                   << H_Xj_num;
  EXPECT_TRUE(H_TBL.isApprox(H_TBL_num, kTolerance)) << "H_TBL mismatch:\nAnalytical:\n"
                                                     << H_TBL << "\nNumerical:\n"
                                                     << H_TBL_num;
}

// ---------------------------------------------------------------------------
// Test: Zero error when measurement is consistent with state
// ---------------------------------------------------------------------------
TEST(LidarExtrinsicsFactor, ZeroErrorAtPerfectMeasurement) {
  const Pose3 X_i(Rot3::Yaw(1.0), gtsam::Point3(10.0, 5.0, 0.0));
  const Pose3 X_j(Rot3::Yaw(1.3), gtsam::Point3(12.0, 6.0, 0.0));
  const Pose3 T_BL(Rot3::Pitch(0.05), gtsam::Point3(0.5, 0.0, 1.2));

  // Ground truth measurement
  const Pose3 measured_Z = T_BL.inverse() * X_i.inverse() * X_j * T_BL;
  auto factor = makeFactor(measured_Z);

  Vector6 error = factor.evaluateError(X_i, X_j, T_BL, nullptr, nullptr, nullptr);
  EXPECT_NEAR(error.norm(), 0.0, 1e-12);
}

// ---------------------------------------------------------------------------
// Test: Non-zero error when measurement is inconsistent
// ---------------------------------------------------------------------------
TEST(LidarExtrinsicsFactor, NonZeroErrorWithBadMeasurement) {
  const Pose3 X_i(Rot3::Yaw(0.0), gtsam::Point3(0, 0, 0));
  const Pose3 X_j(Rot3::Yaw(0.1), gtsam::Point3(1, 0, 0));
  const Pose3 T_BL = Pose3::Identity();

  // Intentionally wrong measurement
  const Pose3 measured_Z(Rot3::Yaw(0.5), gtsam::Point3(5, 5, 5));
  auto factor = makeFactor(measured_Z);

  Vector6 error = factor.evaluateError(X_i, X_j, T_BL, nullptr, nullptr, nullptr);
  EXPECT_GT(error.norm(), 0.1);
}

// ---------------------------------------------------------------------------
// Test: Full GTSAM linearize-based Jacobian verification
// ---------------------------------------------------------------------------
TEST(LidarExtrinsicsFactor, FactorTestingLinearize) {
  const Pose3 X_i(Rot3::RzRyRx(0.1, 0.2, -0.3), gtsam::Point3(1.0, -2.0, 3.0));
  const Pose3 X_j(Rot3::RzRyRx(-0.1, 0.3, 0.5), gtsam::Point3(4.0, 1.0, -1.0));
  const Pose3 T_BL(Rot3::RzRyRx(0.05, -0.02, 0.1), gtsam::Point3(0.3, 0.0, 1.5));

  const Pose3 measured_Z = T_BL.inverse() * X_i.inverse() * X_j * T_BL;
  auto factor = makeFactor(measured_Z);

  gtsam::Values values;
  values.insert(X(0), X_i);
  values.insert(X(1), X_j);
  values.insert(L(0), T_BL);

  EXPECT_TRUE(gtsam::internal::testFactorJacobians("LidarExtrinsicsFactor", factor, values, kDelta,
                                                   kTolerance));
}

// ---------------------------------------------------------------------------
// Test: Multiple linearization points for robustness
// ---------------------------------------------------------------------------
TEST(LidarExtrinsicsFactor, JacobiansMultipleLinPoints) {
  struct TestCase {
    Pose3 X_i, X_j, T_BL;
  };

  std::vector<TestCase> cases = {
      // Pure translation
      {Pose3(Rot3(), gtsam::Point3(0, 0, 0)), Pose3(Rot3(), gtsam::Point3(2, 0, 0)),
       Pose3(Rot3(), gtsam::Point3(0.5, 0, 1.0))},
      // Pure rotation
      {Pose3(Rot3::Yaw(0.0), gtsam::Point3(0, 0, 0)), Pose3(Rot3::Yaw(0.5), gtsam::Point3(0, 0, 0)),
       Pose3(Rot3::Pitch(0.1), gtsam::Point3(0, 0, 0))},
      // Large motion
      {Pose3(Rot3::RzRyRx(0.1, 0.2, 0.3), gtsam::Point3(100, 200, 50)),
       Pose3(Rot3::RzRyRx(0.4, 0.5, 0.6), gtsam::Point3(105, 198, 51)),
       Pose3(Rot3::RzRyRx(0.02, -0.01, 0.03), gtsam::Point3(0.5, -0.1, 1.8))},
      // Near-singular rotation (close to ±π)
      {Pose3(Rot3::RzRyRx(2.9, 0.1, -0.5), gtsam::Point3(-3, 7, 2)),
       Pose3(Rot3::RzRyRx(-2.8, -0.2, 0.8), gtsam::Point3(1, -4, 5)),
       Pose3(Rot3::RzRyRx(0.1, 0.1, 0.1), gtsam::Point3(0.3, 0.2, 1.5))},
  };

  for (size_t i = 0; i < cases.size(); ++i) {
    const auto& tc = cases[i];
    const Pose3 measured_Z = tc.T_BL.inverse() * tc.X_i.inverse() * tc.X_j * tc.T_BL;
    auto factor = makeFactor(measured_Z);

    Matrix H_Xi, H_Xj, H_TBL;
    factor.evaluateError(tc.X_i, tc.X_j, tc.T_BL, &H_Xi, &H_Xj, &H_TBL);

    auto errorFn = [&](const Pose3& xi, const Pose3& xj, const Pose3& tbl) {
      return factor.evaluateError(xi, xj, tbl, nullptr, nullptr, nullptr);
    };

    Matrix H_Xi_num = gtsam::numericalDerivative31<Vector6, Pose3, Pose3, Pose3>(
        errorFn, tc.X_i, tc.X_j, tc.T_BL, kDelta);
    Matrix H_Xj_num = gtsam::numericalDerivative32<Vector6, Pose3, Pose3, Pose3>(
        errorFn, tc.X_i, tc.X_j, tc.T_BL, kDelta);
    Matrix H_TBL_num = gtsam::numericalDerivative33<Vector6, Pose3, Pose3, Pose3>(
        errorFn, tc.X_i, tc.X_j, tc.T_BL, kDelta);

    EXPECT_TRUE(H_Xi.isApprox(H_Xi_num, kTolerance))
        << "Case " << i << " H_Xi mismatch:\nAnalytical:\n"
        << H_Xi << "\nNumerical:\n"
        << H_Xi_num;
    EXPECT_TRUE(H_Xj.isApprox(H_Xj_num, kTolerance))
        << "Case " << i << " H_Xj mismatch:\nAnalytical:\n"
        << H_Xj << "\nNumerical:\n"
        << H_Xj_num;
    EXPECT_TRUE(H_TBL.isApprox(H_TBL_num, kTolerance))
        << "Case " << i << " H_TBL mismatch:\nAnalytical:\n"
        << H_TBL << "\nNumerical:\n"
        << H_TBL_num;
  }
}

// ---------------------------------------------------------------------------
// Test: Symmetry — swapping X_i and X_j with inverted measurement
// ---------------------------------------------------------------------------
TEST(LidarExtrinsicsFactor, SwapPosesInvertMeasurement) {
  const Pose3 X_i(Rot3::Yaw(0.3), gtsam::Point3(1, 2, 0));
  const Pose3 X_j(Rot3::Yaw(0.6), gtsam::Point3(4, 3, 0));
  const Pose3 T_BL(Rot3::Roll(0.05), gtsam::Point3(0.5, 0, 1.2));

  // Forward measurement
  const Pose3 Z_ij = T_BL.inverse() * X_i.inverse() * X_j * T_BL;

  // Reverse factor: swap i,j with inverted measurement
  const Pose3 Z_ji = T_BL.inverse() * X_j.inverse() * X_i * T_BL;

  nufuse::factor::LidarExtrinsicsFactor factor_fwd(X(0), X(1), L(0), Z_ij,
                                                   gtsam::noiseModel::Unit::Create(6));
  nufuse::factor::LidarExtrinsicsFactor factor_rev(X(1), X(0), L(0), Z_ji,
                                                   gtsam::noiseModel::Unit::Create(6));

  // Both should have zero error at the correct state
  Vector6 err_fwd = factor_fwd.evaluateError(X_i, X_j, T_BL, nullptr, nullptr, nullptr);
  Vector6 err_rev = factor_rev.evaluateError(X_j, X_i, T_BL, nullptr, nullptr, nullptr);

  EXPECT_NEAR(err_fwd.norm(), 0.0, 1e-12);
  EXPECT_NEAR(err_rev.norm(), 0.0, 1e-12);
}

// ---------------------------------------------------------------------------
// Test: Extrinsics at identity reduces to BetweenFactor behavior
// ---------------------------------------------------------------------------
TEST(LidarExtrinsicsFactor, IdentityExtrinsicsIsBetween) {
  const Pose3 X_i(Rot3::Yaw(0.2), gtsam::Point3(1, 0, 0));
  const Pose3 X_j(Rot3::Yaw(0.4), gtsam::Point3(3, 1, 0));
  const Pose3 T_BL = Pose3::Identity();

  // With T_BL = I, measurement should equal X_i.between(X_j)
  const Pose3 between_ij = X_i.between(X_j);
  auto factor = makeFactor(between_ij);

  Vector6 error = factor.evaluateError(X_i, X_j, T_BL, nullptr, nullptr, nullptr);
  EXPECT_NEAR(error.norm(), 0.0, 1e-12);

  // Jacobians should still be correct
  gtsam::Values values;
  values.insert(X(0), X_i);
  values.insert(X(1), X_j);
  values.insert(L(0), T_BL);

  EXPECT_TRUE(gtsam::internal::testFactorJacobians("LidarExtrinsicsFactor_IdentityExtrinsics",
                                                   factor, values, kDelta, kTolerance));
}
