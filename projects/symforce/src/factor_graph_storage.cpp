/// @file factor_graph_storage.cpp
/// @brief CFactorGraphStorage — factor-creation methods + explicit instantiations.

#include "factor_graph_storage.hpp"

template <typename G>
void CFactorGraphStorage<G>::addPrior(const typename G::PriorMeas& m) {
  factors_.push_back(G::buildPriorFactor(values_, next_prior_++, m));
}

template <typename G>
void CFactorGraphStorage<G>::addBetween(const typename G::BetweenMeas& m) {
  factors_.push_back(G::buildBetweenFactor(values_, next_edge_++, m));
}

template <typename G>
void CFactorGraphStorage<G>::addLoopClosure(
    const typename G::LoopClosureMeas& m) {
  factors_.push_back(G::buildLoopClosureFactor(values_, next_loop_++, m));
}

template <typename G>
void CFactorGraphStorage<G>::addLandmarkObservation(
    const typename G::LandmarkMeas& m) {
  factors_.push_back(G::buildLandmarkFactor(values_, next_lm_obs_++, m));
}

// ---- Explicit instantiations --------------------------------------------
template class CFactorGraphStorage<SE2>;
template class CFactorGraphStorage<SE3>;
