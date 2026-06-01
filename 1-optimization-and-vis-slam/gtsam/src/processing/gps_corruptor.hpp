/// @file processing/gps_corruptor.hpp
/// @brief GPS corruption injection for robustness testing.
///
/// Separates test-harness concern from production measurement processing.

#pragma once

#include <random>
#include <vector>

#include "domain/measurements.hpp"

namespace nufuse::processing {

/// @brief Configuration for GPS spike injection.
struct GpsCorruptorConfig {
  double spike_metres = 100.0;
  double spike_fraction = 0.20;
  unsigned rng_seed = 42u;
};

/// @brief Result of corruption: the fix (possibly modified) and whether it was corrupted.
struct CorruptedGnssFix {
  domain::GnssFix fix;
  bool corrupted = false;
};

/// @brief Applies random position spikes to a fraction of GNSS fixes.
/// Returns a new vector with the same fixes, some offset by spike_metres.
/// The corruption flag allows downstream tracking of injected outliers.
std::vector<CorruptedGnssFix> corruptGnss(const std::vector<domain::GnssFix>& fixes,
                                          const GpsCorruptorConfig& cfg = {});

}  // namespace nufuse::processing
