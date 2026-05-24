/// @file factor_storage.hpp
/// @brief Plain data container holding all factors and initial estimates.
///
/// FactorStorage is the output of measurement processing and the input to
/// GTSAM graph construction.  It owns no graph logic — just typed vectors.

#pragma once

#include "factor_types.hpp"

#include <optional>
#include <vector>

/// @brief Collects all factors and keyframe estimates produced by measurement
///        processing, ready for graph construction.
struct FactorStorage {
    std::optional<PriorFactor> prior;
    std::vector<StoredImuFactor> imu_factors;
    std::vector<StoredGpsFactor> gps_factors;
    std::vector<StoredLidarFactor> lidar_factors;
    std::vector<KeyframeEstimate> estimates;
    int num_corrupted = 0;
};
