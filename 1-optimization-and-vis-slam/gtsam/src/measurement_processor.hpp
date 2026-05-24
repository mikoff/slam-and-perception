/// @file measurement_processor.hpp
/// @brief Incremental measurement processor that builds a FactorStorage.
///
/// Accepts raw IMU, GNSS, and LiDAR measurements one at a time in chronological
/// order.  Performs IMU preintegration, WGS-84 → ENU conversion, and odometry
/// interpolation for initial values.  The output is a FactorStorage ready for
/// graph construction.

#pragma once

#include "factor_storage.hpp"
#include "nuscenes_domain_types.hpp"

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>

#include <optional>
#include <random>
#include <vector>

// ─── Configuration ───────────────────────────────────────────────────────────

/// @brief Controls GPS corruption injection for robustness testing.
struct ProcessorConfig {
    double spike_metres   = 100.0;
    double spike_fraction = 0.20;
    unsigned rng_seed     = 42u;
};

// ─── Measurement processor ───────────────────────────────────────────────────

/// @brief Processes raw sensor measurements into a FactorStorage.
///
/// Feed measurements via addImu() / addGnss() / addLidar() in chronological
/// order.  Call finalize() to retrieve the completed FactorStorage.
class MeasurementProcessor {
public:
    /// @param cfg  Corruption / noise configuration.
    /// @param odom Full odometry stream used for pose interpolation only.
    MeasurementProcessor(const ProcessorConfig& cfg,
                         const std::vector<OdomMeasurement>& odom);

    /// @brief Integrate one IMU sample into the running preintegration window.
    void addImu(const ImuMeasurement& m);

    /// @brief Process a GNSS fix.  The first fix establishes the ENU reference.
    void addGnss(const GnssFix& fix);

    /// @brief Process a LiDAR scan timestamp with an optional body-frame
    ///        relative pose from the previous LiDAR keyframe.
    void addLidar(uint64_t stamp_ns, std::optional<gtsam::Pose3> rel_pose);

    /// @brief Consume the processor and return the completed FactorStorage.
    FactorStorage finalize() &&;

private:
    void createKeyframe(uint64_t stamp_ns);

    const ProcessorConfig& cfg_;
    const std::vector<OdomMeasurement>& odom_;
    gtsam::imuBias::ConstantBias bias0_;
    gtsam::PreintegratedCombinedMeasurements pim_;
    FactorStorage storage_;
    std::mt19937 rng_;
    std::bernoulli_distribution coin_;
    std::uniform_real_distribution<double> spike_dir_{-1.0, 1.0};
    std::optional<ImuMeasurement> prev_imu_;
    double lat0_ = 0.0, lon0_ = 0.0, alt0_ = 0.0;
    gtsam::Point3 odom_origin_;
    int idx_ = -1;
    int lidar_prev_idx_ = -1;
};

// ─── Utilities ───────────────────────────────────────────────────────────────

/// @brief Interpolate odometry at the given timestamp using SE(3) geodesic
///        interpolation and linear lerp for velocity/acceleration.
///
/// @param odom     Odometry samples sorted by ascending timestamp.
/// @param stamp_ns Query timestamp [ns].
OdomMeasurement interpolateOdom(const std::vector<OdomMeasurement>& odom,
                                uint64_t stamp_ns);

/// @brief Convert WGS-84 geodetic coordinates to local ENU.
gtsam::Point3 llaToEnu(double lat, double lon, double alt,
                        double lat0, double lon0, double alt0);
