#pragma once

/// @file geometry_types.hpp
/// @brief Geometry policies (SE2, SE3) and typed measurement structs.

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

#include <sym/pose2.h>
#include <sym/pose3.h>

#include <symforce/opt/factor.h>
#include <symforce/opt/values.h>

#include <slam/core/strong_id.hpp>

// -- Type-safe identifiers ------------------------------------------------
struct PoseTag {};
struct LandmarkTag {};
using PoseId     = slam::core::StrongId<PoseTag, int>;
using LandmarkId = slam::core::StrongId<LandmarkTag, int>;

// -- Forward declarations for measurement structs -------------------------

template <typename G> struct PriorMeasurement;
template <typename G> struct BetweenMeasurement;
template <typename G> struct LoopClosureMeasurement;
template <typename G> struct LandmarkMeasurement;

// =========================================================================
/// @brief SE2 geometry policy for 2D pose-graph SLAM.
// =========================================================================
struct SE2 {
  using Pose               = sym::Pose2d;
  using Position           = Eigen::Vector2d;
  using PoseInfoMatrix     = Eigen::Matrix3d;
  using LandmarkInfoMatrix = Eigen::Matrix2d;

  using PriorMeas       = PriorMeasurement<SE2>;
  using BetweenMeas     = BetweenMeasurement<SE2>;
  using LoopClosureMeas = LoopClosureMeasurement<SE2>;
  using LandmarkMeas    = LandmarkMeasurement<SE2>;

  /// @brief Store measurement values and build a prior factor.
  static sym::Factor<double> buildPriorFactor(
      sym::Values<double>& v, int index, const PriorMeas& m);

  /// @brief Store measurement values and build a between (odometry) factor.
  static sym::Factor<double> buildBetweenFactor(
      sym::Values<double>& v, int index, const BetweenMeas& m);

  /// @brief Store measurement values and build a robust loop-closure factor.
  static sym::Factor<double> buildLoopClosureFactor(
      sym::Values<double>& v, int index, const LoopClosureMeas& m);

  /// @brief Store measurement values and build a landmark observation factor.
  static sym::Factor<double> buildLandmarkFactor(
      sym::Values<double>& v, int index, const LandmarkMeas& m);

  /// @brief Serialize a pose to JSON.
  static nlohmann::json poseToJson(PoseId id, const Pose& pose);

  /// @brief Serialize a landmark position to JSON.
  static nlohmann::json landmarkToJson(LandmarkId id, const Position& pos);
};

// =========================================================================
/// @brief SE3 geometry policy for 3D pose-graph SLAM.
// =========================================================================
struct SE3 {
  using Pose               = sym::Pose3d;
  using Position           = Eigen::Vector3d;
  using PoseInfoMatrix     = Eigen::Matrix<double, 6, 6>;
  using LandmarkInfoMatrix = Eigen::Matrix3d;

  using PriorMeas       = PriorMeasurement<SE3>;
  using BetweenMeas     = BetweenMeasurement<SE3>;
  using LoopClosureMeas = LoopClosureMeasurement<SE3>;
  using LandmarkMeas    = LandmarkMeasurement<SE3>;

  /// @brief Store measurement values and build a prior factor.
  static sym::Factor<double> buildPriorFactor(
      sym::Values<double>& v, int index, const PriorMeas& m);

  /// @brief Store measurement values and build a between (odometry) factor.
  static sym::Factor<double> buildBetweenFactor(
      sym::Values<double>& v, int index, const BetweenMeas& m);

  /// @brief Store measurement values and build a robust loop-closure factor.
  static sym::Factor<double> buildLoopClosureFactor(
      sym::Values<double>& v, int index, const LoopClosureMeas& m);

  /// @brief Store measurement values and build a landmark observation factor.
  static sym::Factor<double> buildLandmarkFactor(
      sym::Values<double>& v, int index, const LandmarkMeas& m);

  /// @brief Serialize a pose to JSON.
  static nlohmann::json poseToJson(PoseId id, const Pose& pose);

  /// @brief Serialize a landmark position to JSON.
  static nlohmann::json landmarkToJson(LandmarkId id, const Position& pos);
};

// =========================================================================
// Measurement structs — templated on geometry policy G
// =========================================================================

/// @brief Prior factor measurement data.
template <typename G>
struct PriorMeasurement {
  PoseId pose_id;
  typename G::Pose prior_pose;
  typename G::PoseInfoMatrix info;
};

/// @brief Odometry (between) factor measurement data.
template <typename G>
struct BetweenMeasurement {
  PoseId id_from;
  PoseId id_to;
  typename G::Pose relative_pose;
  typename G::PoseInfoMatrix info;
};

/// @brief Robust loop-closure factor measurement data.
template <typename G>
struct LoopClosureMeasurement {
  PoseId id_from;
  PoseId id_to;
  typename G::Pose relative_pose;
  typename G::PoseInfoMatrix info;
  double barron_alpha = -2.0;
  double barron_delta = 1.0;
};

/// @brief Landmark observation factor measurement data.
template <typename G>
struct LandmarkMeasurement {
  PoseId pose_id;
  LandmarkId landmark_id;
  typename G::Position observation;
  typename G::LandmarkInfoMatrix info;
  double barron_alpha = -2.0;
  double barron_delta = 1.0;
};
