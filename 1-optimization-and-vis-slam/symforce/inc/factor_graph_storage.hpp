#pragma once

/// @file factor_graph_storage.hpp
/// @brief Manages sym::Values and sym::Factor containers for pose-graph SLAM.

#include <unordered_set>
#include <vector>

#include <symforce/opt/factor.h>
#include <symforce/opt/values.h>

#include "geometry_types.hpp"

/// @brief Typed factor-graph storage, templated on geometry policy G.
///
/// Tracks initialized poses/landmarks, validates references, and delegates
/// factor construction to the geometry policy.
template <typename G>
class CFactorGraphStorage {
 public:
  CFactorGraphStorage();

  /// @brief Set initial pose estimate. Must be called before referencing.
  void setPose(PoseId id, const typename G::Pose& pose);

  /// @brief Check whether pose @p id has been initialized.
  bool hasPose(PoseId id) const;

  /// @brief Number of pose slots (max_id + 1).
  int numPoses() const;

  /// @brief Set initial landmark estimate. Must be called before referencing.
  void setLandmark(LandmarkId id, const typename G::Position& position);

  /// @brief Check whether landmark @p id has been initialized.
  bool hasLandmark(LandmarkId id) const;

  /// @brief All initialized landmark ids.
  const std::unordered_set<LandmarkId>& landmarkIds() const;

  /// @brief Add a prior factor. Pose must be initialized.
  void addPrior(const typename G::PriorMeas& m);

  /// @brief Add a between (odometry) factor. Both poses must be initialized.
  void addBetween(const typename G::BetweenMeas& m);

  /// @brief Add a robust loop-closure factor. Both poses must be initialized.
  void addLoopClosure(const typename G::LoopClosureMeas& m);

  /// @brief Add a landmark observation. Pose and landmark must be initialized.
  void addLandmarkObservation(const typename G::LandmarkMeas& m);

  /// @brief Mutable access to sym::Values.
  sym::Values<double>& values();

  /// @brief Const access to sym::Values.
  const sym::Values<double>& values() const;

  /// @brief All registered factors.
  const std::vector<sym::Factor<double>>& factors() const;

  int numEdges() const;
  int numLoopClosures() const;
  int numLandmarkObservations() const;
  int numPriors() const;
  int numFactors() const;

 private:
  sym::Values<double> values_;
  std::vector<sym::Factor<double>> factors_;

  std::unordered_set<PoseId> poses_;
  std::unordered_set<LandmarkId> landmarks_;
  int max_pose_id_ = -1;

  int next_edge_ = 0;
  int next_loop_ = 0;
  int next_lm_obs_ = 0;
  int next_prior_ = 0;
};

#include "factor_graph_storage.inl"
