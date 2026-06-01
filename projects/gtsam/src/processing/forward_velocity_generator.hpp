/// @file processing/forward_velocity_generator.hpp
/// @brief Forward velocity scale factor generation from keyframe pairs.

#pragma once

#include <vector>

#include "domain/measurements.hpp"
#include "graph/factor_storage.hpp"

namespace nufuse::processing {

/// @brief Generate forward velocity scale factors between consecutive keyframes.
/// Each factor constrains the forward displacement to match scaled odometry speed.
std::vector<graph::StoredForwardVelocityFactor> generateForwardVelocityFactors(
    const graph::FactorStorage& storage, const std::vector<domain::OdomMeasurement>& odom);

}  // namespace nufuse::processing
