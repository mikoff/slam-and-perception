/// @file processing/forward_velocity_generator.cpp
/// @brief Implementation of forward velocity factor generation.

#include "processing/forward_velocity_generator.hpp"

#include "core/gtsam_boundary.hpp"
#include "processing/geo_utils.hpp"

namespace nufuse::processing {

std::vector<graph::StoredForwardVelocityFactor> generateForwardVelocityFactors(
    const graph::FactorStorage& storage, const std::vector<domain::OdomMeasurement>& odom) {
  std::vector<graph::StoredForwardVelocityFactor> factors;
  if (odom.empty() || storage.estimates.size() < 2) return factors;
  factors.reserve(storage.estimates.size() - 1);

  for (size_t i = 0; i + 1 < storage.estimates.size(); ++i) {
    const auto& est_from = storage.estimates[i];
    const auto& est_to = storage.estimates[i + 1];

    const uint64_t stamp_from = est_from.stamp.value();
    const uint64_t stamp_to = est_to.stamp.value();
    const double dt = static_cast<double>(stamp_to - stamp_from) * 1e-9;
    if (dt < 1e-6) continue;

    const auto& nearest = findNearestOdom(odom, stamp_from);
    const double forward_speed = nearest.velocity.value().x();

    graph::StoredForwardVelocityFactor f;
    f.stamp_from = est_from.stamp;
    f.stamp_to = est_to.stamp;
    f.measured_v = forward_speed;
    f.dt = dt;
    f.omega_B = core::AngularVelocityBody(nearest.angular_velocity.value());
    factors.push_back(f);
  }
  return factors;
}

}  // namespace nufuse::processing
