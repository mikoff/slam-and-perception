/// @file processing/nhc_generator.cpp
/// @brief Implementation of NHC factor generation.

#include "processing/nhc_generator.hpp"

#include "core/gtsam_boundary.hpp"
#include "processing/geo_utils.hpp"

namespace nufuse::processing {

std::vector<graph::StoredNhcFactor> generateNhcFactors(
    const graph::FactorStorage& storage, const std::vector<domain::OdomMeasurement>& odom) {
  std::vector<graph::StoredNhcFactor> nhc_factors;
  if (odom.empty()) return nhc_factors;
  nhc_factors.reserve(storage.estimates.size());

  for (const auto& est : storage.estimates) {
    const auto& nearest = findNearestOdom(odom, est.stamp.value());
    graph::StoredNhcFactor nhc;
    nhc.stamp = est.stamp;
    nhc.omega_B = core::AngularVelocityBody(nearest.angular_velocity.value());
    nhc_factors.push_back(nhc);
  }
  return nhc_factors;
}

}  // namespace nufuse::processing
