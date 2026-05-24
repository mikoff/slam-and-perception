/// @file processing/measurement_processor.hpp
/// @brief Incremental measurement processor producing factor storage.

#pragma once

#include "domain/measurements.hpp"
#include "graph/factor_storage.hpp"

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>

#include <optional>
#include <random>
#include <vector>

namespace nufuse::processing {

/// @brief Controls GPS corruption injection for robustness testing.
struct ProcessorConfig {
    double spike_metres   = 100.0;
    double spike_fraction = 0.20;
    unsigned rng_seed     = 42u;
};

/// @brief Processes raw sensor measurements into a FactorStorage.
///
/// Feed measurements via addImu/addGnss/addLidar in chronological order.
/// Call finalize() to retrieve the completed storage.
class MeasurementProcessor {
public:
    MeasurementProcessor(const ProcessorConfig& cfg,
                         const std::vector<domain::OdomMeasurement>& odom);

    void addImu(const domain::ImuMeasurement& m);
    void addGnss(const domain::GnssFix& fix);
    void addLidar(core::Timestamp stamp, std::optional<gtsam::Pose3> rel_pose);

    graph::FactorStorage finalize() &&;

private:
    void createKeyframe(uint64_t stamp_ns);
    void initializeFirstKeyframe(uint64_t stamp_ns);
    void storeImuFactor(uint64_t stamp_ns);
    void storeKeyframeEstimate(uint64_t stamp_ns);

    const ProcessorConfig& cfg_;
    const std::vector<domain::OdomMeasurement>& odom_;
    gtsam::imuBias::ConstantBias bias0_;
    gtsam::PreintegratedCombinedMeasurements pim_;
    graph::FactorStorage storage_;

    std::mt19937 rng_;
    std::bernoulli_distribution coin_;
    std::uniform_real_distribution<double> spike_dir_{-1.0, 1.0};

    std::optional<domain::ImuMeasurement> prev_imu_;
    double lat0_ = 0.0, lon0_ = 0.0, alt0_ = 0.0;
    gtsam::Point3 odom_origin_;
    int idx_ = -1;
    int lidar_prev_idx_ = -1;
};

}  // namespace nufuse::processing
