/// @file factor_graph.inl
/// @brief Inline definitions for CFactorGraph<G>.

template <typename G>
inline CFactorGraphStorage<G>& CFactorGraph<G>::storage() {
  return storage_;
}

template <typename G>
inline const CFactorGraphStorage<G>& CFactorGraph<G>::storage() const {
  return storage_;
}
