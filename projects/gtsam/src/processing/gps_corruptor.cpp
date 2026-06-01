/// @file processing/gps_corruptor.cpp
/// @brief Implementation of GPS corruption injection.

#include "processing/gps_corruptor.hpp"

namespace nufuse::processing {

std::vector<CorruptedGnssFix> corruptGnss(const std::vector<domain::GnssFix>& fixes,
                                          const GpsCorruptorConfig& cfg) {
  std::mt19937 rng(cfg.rng_seed);
  std::bernoulli_distribution coin(cfg.spike_fraction);
  std::uniform_real_distribution<double> spike_dir(-1.0, 1.0);

  std::vector<CorruptedGnssFix> result;
  result.reserve(fixes.size());

  for (const auto& fix : fixes) {
    CorruptedGnssFix out;
    out.fix = fix;
    out.corrupted = coin(rng);
    if (out.corrupted) {
      // Inject spike in LLA space (approximate — sufficient for testing)
      auto& lla = out.fix.lla;
      constexpr double kDeg2Rad = M_PI / 180.0;
      constexpr double kR = 6'378'137.0;
      const double lat_rad = lla.value()(0) * kDeg2Rad;
      const double metres_per_deg_lat = kDeg2Rad * kR;
      const double metres_per_deg_lon = kDeg2Rad * kR * std::cos(lat_rad);

      gtsam::Vector3 offset;
      offset(0) = spike_dir(rng) * cfg.spike_metres / metres_per_deg_lat;
      offset(1) = spike_dir(rng) * cfg.spike_metres / metres_per_deg_lon;
      offset(2) = spike_dir(rng) * cfg.spike_metres;
      out.fix.lla = core::Wgs84Lla(lla.value() + offset);
    }
    result.push_back(std::move(out));
  }

  return result;
}

}  // namespace nufuse::processing
