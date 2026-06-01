/// @file processing/nhc_generator.hpp
/// @brief Non-holonomic constraint factor generation from keyframe estimates.

#pragma once

#include <vector>

#include "domain/measurements.hpp"
#include "graph/factor_storage.hpp"

namespace nufuse::processing {

/// @brief Generate non-holonomic constraint factors for all keyframes.
/// Each factor constrains lateral/vertical velocity at the rear axle to zero.
std::vector<graph::StoredNhcFactor> generateNhcFactors(
    const graph::FactorStorage& storage, const std::vector<domain::OdomMeasurement>& odom);

}  // namespace nufuse::processing
