/// @file factor_graph_storage.inl
/// @brief Inline definitions for CFactorGraphStorage<G>.

#include <sym/util/epsilon.h>

template <typename G>
inline CFactorGraphStorage<G>::CFactorGraphStorage() {
  values_.Set('e', sym::kDefaultEpsilond);
}

template <typename G>
inline void CFactorGraphStorage<G>::setPose(
    PoseId id, const typename G::Pose& pose) {
  values_.Set({'P', id.value()}, pose);
  poses_.insert(id);
  max_pose_id_ = std::max(max_pose_id_, id.value());
}

template <typename G>
inline bool CFactorGraphStorage<G>::hasPose(PoseId id) const {
  return poses_.count(id) > 0;
}

template <typename G>
inline int CFactorGraphStorage<G>::numPoses() const {
  return max_pose_id_ + 1;
}

template <typename G>
inline void CFactorGraphStorage<G>::setLandmark(
    LandmarkId id, const typename G::Position& position) {
  values_.Set({'L', id.value()}, position);
  landmarks_.insert(id);
}

template <typename G>
inline bool CFactorGraphStorage<G>::hasLandmark(LandmarkId id) const {
  return landmarks_.count(id) > 0;
}

template <typename G>
inline const std::unordered_set<LandmarkId>&
CFactorGraphStorage<G>::landmarkIds() const {
  return landmarks_;
}

template <typename G>
inline sym::Values<double>& CFactorGraphStorage<G>::values() {
  return values_;
}

template <typename G>
inline const sym::Values<double>&
CFactorGraphStorage<G>::values() const {
  return values_;
}

template <typename G>
inline const std::vector<sym::Factor<double>>&
CFactorGraphStorage<G>::factors() const {
  return factors_;
}

template <typename G>
inline int CFactorGraphStorage<G>::numEdges() const { return next_edge_; }

template <typename G>
inline int CFactorGraphStorage<G>::numLoopClosures() const {
  return next_loop_;
}

template <typename G>
inline int CFactorGraphStorage<G>::numLandmarkObservations() const {
  return next_lm_obs_;
}

template <typename G>
inline int CFactorGraphStorage<G>::numPriors() const { return next_prior_; }

template <typename G>
inline int CFactorGraphStorage<G>::numFactors() const {
  return static_cast<int>(factors_.size());
}


