/// @file geometry_types.cpp
/// @brief SE2 and SE3 policy definitions — factor builders and JSON serializers.

#include "geometry_types.hpp"

#include <cmath>

#include <symforce/opt/key.h>

#include <symforce/sym/between_factor_se2_factor.h>
#include <symforce/sym/loop_closure_factor_se2_factor.h>
#include <symforce/sym/pose_to_landmark_factor_se2_factor.h>
#include <symforce/sym/prior_factor_se2_factor.h>

#include <symforce/sym/between_factor_se3_factor.h>
#include <symforce/sym/loop_closure_factor_se3_factor.h>
#include <symforce/sym/pose_to_landmark_factor_se3_factor.h>
#include <symforce/sym/prior_se3_factor.h>

namespace {

/// @brief Permute [x,y,θ] info matrix to [θ,x,y] and compute upper Cholesky.
Eigen::Matrix3d permutedSqrtInfo3(const Eigen::Matrix3d& info_xyt) {
  static const Eigen::Matrix3d P =
      (Eigen::Matrix3d() << 0, 0, 1, 1, 0, 0, 0, 1, 0).finished();
  Eigen::Matrix3d info_txy = P * info_xyt * P.transpose();
  return info_txy.llt().matrixL().transpose();
}

/// @brief Upper Cholesky of a 2×2 information matrix.
Eigen::Matrix2d sqrtInfo2(const Eigen::Matrix2d& info) {
  return info.llt().matrixL().transpose();
}

}  // namespace

// =========================================================================
//  SE2
// =========================================================================

sym::Factor<double> SE2::buildPriorFactor(
    sym::Values<double>& v, int k, const PriorMeas& m) {
  v.Set({'q', k}, m.prior_pose);
  v.Set({'Q', k}, permutedSqrtInfo3(m.info));
  return sym::Factor<double>::Hessian(
      sym::PriorFactorSe2Factor<double>,
      {{'P', m.pose_id.value()}, {'q', k}, {'Q', k}, {'e'}},
      {{'P', m.pose_id.value()}});
}

sym::Factor<double> SE2::buildBetweenFactor(
    sym::Values<double>& v, int k, const BetweenMeas& m) {
  v.Set({'m', k}, m.relative_pose);
  v.Set({'I', k}, permutedSqrtInfo3(m.info));
  return sym::Factor<double>::Hessian(
      sym::BetweenFactorSe2Factor<double>,
      {{'P', m.id_from.value()}, {'P', m.id_to.value()}, {'m', k}, {'I', k}, {'e'}},
      {{'P', m.id_from.value()}, {'P', m.id_to.value()}});
}

sym::Factor<double> SE2::buildLoopClosureFactor(
    sym::Values<double>& v, int k, const LoopClosureMeas& m) {
  v.Set({'c', k}, m.relative_pose);
  v.Set({'C', k}, permutedSqrtInfo3(m.info));
  v.Set({'a', k}, m.barron_alpha);
  v.Set({'d', k}, m.barron_delta);
  return sym::Factor<double>::Hessian(
      sym::LoopClosureFactorSe2Factor<double>,
      {{'P', m.id_from.value()}, {'P', m.id_to.value()}, {'c', k}, {'C', k},
       {'a', k}, {'d', k}, {'e'}},
      {{'P', m.id_from.value()}, {'P', m.id_to.value()}});
}

sym::Factor<double> SE2::buildLandmarkFactor(
    sym::Values<double>& v, int k, const LandmarkMeas& m) {
  v.Set({'o', k}, m.observation);
  v.Set({'J', k}, sqrtInfo2(m.info));
  v.Set({'b', k}, m.barron_alpha);
  v.Set({'B', k}, m.barron_delta);
  return sym::Factor<double>::Hessian(
      sym::PoseToLandmarkFactorSe2Factor<double>,
      {{'P', m.pose_id.value()}, {'o', k}, {'L', m.landmark_id.value()}, {'J', k},
       {'b', k}, {'B', k}, {'e'}},
      {{'P', m.pose_id.value()}, {'L', m.landmark_id.value()}});
}

nlohmann::json SE2::poseToJson(PoseId id, const Pose& pose) {
  const auto& d = pose.Data();
  return {{"id", id.value()},
          {"x", d[2]}, {"y", d[3]},
          {"theta", std::atan2(d[1], d[0])}};
}

nlohmann::json SE2::landmarkToJson(LandmarkId id, const Position& pos) {
  return {{"id", id.value()}, {"x", pos[0]}, {"y", pos[1]}};
}

// =========================================================================
//  SE3
// =========================================================================

sym::Factor<double> SE3::buildPriorFactor(
    sym::Values<double>& v, int k, const PriorMeas& m) {
  v.Set({'q', k}, m.prior_pose);
  v.Set({'Q', k}, m.info.diagonal().cwiseSqrt());
  return sym::Factor<double>::Hessian(
      sym::PriorSe3Factor<double>,
      {{'P', m.pose_id.value()}, {'q', k}, {'Q', k}, {'e'}},
      {{'P', m.pose_id.value()}});
}

sym::Factor<double> SE3::buildBetweenFactor(
    sym::Values<double>& v, int k, const BetweenMeas& m) {
  v.Set({'m', k}, m.relative_pose);
  v.Set({'I', k}, m.info.diagonal().cwiseSqrt());
  return sym::Factor<double>::Hessian(
      sym::BetweenFactorSe3Factor<double>,
      {{'P', m.id_from.value()}, {'P', m.id_to.value()}, {'m', k}, {'I', k}, {'e'}},
      {{'P', m.id_from.value()}, {'P', m.id_to.value()}});
}

sym::Factor<double> SE3::buildLoopClosureFactor(
    sym::Values<double>& v, int k, const LoopClosureMeas& m) {
  v.Set({'c', k}, m.relative_pose);
  v.Set({'C', k}, m.info.diagonal().cwiseSqrt());
  v.Set({'a', k}, m.barron_alpha);
  v.Set({'d', k}, m.barron_delta);
  return sym::Factor<double>::Hessian(
      sym::LoopClosureFactorSe3Factor<double>,
      {{'P', m.id_from.value()}, {'P', m.id_to.value()}, {'c', k}, {'C', k},
       {'a', k}, {'d', k}, {'e'}},
      {{'P', m.id_from.value()}, {'P', m.id_to.value()}});
}

sym::Factor<double> SE3::buildLandmarkFactor(
    sym::Values<double>& v, int k, const LandmarkMeas& m) {
  v.Set({'o', k}, m.observation);
  v.Set({'J', k}, m.info.diagonal().cwiseSqrt());
  v.Set({'b', k}, m.barron_alpha);
  v.Set({'B', k}, m.barron_delta);
  return sym::Factor<double>::Hessian(
      sym::PoseToLandmarkFactorSe3Factor<double>,
      {{'P', m.pose_id.value()}, {'o', k}, {'L', m.landmark_id.value()}, {'J', k},
       {'b', k}, {'B', k}, {'e'}},
      {{'P', m.pose_id.value()}, {'L', m.landmark_id.value()}});
}

nlohmann::json SE3::poseToJson(PoseId id, const Pose& pose) {
  const auto& d = pose.Data();
  return {{"id", id.value()},
          {"x", d[4]}, {"y", d[5]}, {"z", d[6]},
          {"qx", d[0]}, {"qy", d[1]}, {"qz", d[2]}, {"qw", d[3]}};
}

nlohmann::json SE3::landmarkToJson(LandmarkId id, const Position& pos) {
  return {{"id", id.value()}, {"x", pos[0]}, {"y", pos[1]}, {"z", pos[2]}};
}
