/// @file processing/measurement_processor.cpp
/// @brief Implementation of MeasurementProcessor.

#include "processing/measurement_processor.hpp"
#include "processing/geo_utils.hpp"

#include <cmath>

namespace nufuse::processing {

using namespace gtsam;

// ─── IMU parameters (NuScenes Bosch BMI055) ──────────────────────────────────

static auto makeImuParams() {
    auto p = PreintegratedCombinedMeasurements::Params::MakeSharedU(9.81);
    p->accelerometerCovariance = I_3x3 * std::pow(0.0003924, 2);
    p->gyroscopeCovariance     = I_3x3 * std::pow(0.000205689, 2);
    p->integrationCovariance   = I_3x3 * 1e-8;
    p->biasAccCovariance       = I_3x3 * std::pow(0.004905, 2);
    p->biasOmegaCovariance     = I_3x3 * std::pow(1.4544e-6, 2);
    return p;
}

// ─── Construction ────────────────────────────────────────────────────────────

MeasurementProcessor::MeasurementProcessor(
    const ProcessorConfig& cfg, const std::vector<domain::OdomMeasurement>& odom)
    : cfg_(cfg)
    , odom_(odom)
    , pim_(makeImuParams(), imuBias::ConstantBias{})
    , rng_(cfg.rng_seed)
    , coin_(cfg.spike_fraction) {}

// ─── IMU integration ─────────────────────────────────────────────────────────

void MeasurementProcessor::addImu(const domain::ImuMeasurement& m) {
    if (prev_imu_) {
        const double dt = static_cast<double>(
            m.stamp.value() - prev_imu_->stamp.value()) * 1e-9;
        if (dt > 0.0 && dt < 1.0) {
            pim_.integrateMeasurement(
                prev_imu_->linear_acceleration.value(),
                prev_imu_->angular_velocity.value(), dt);
        }
    }
    prev_imu_ = m;
}

// ─── First keyframe initialization ──────────────────────────────────────────

void MeasurementProcessor::initializeFirstKeyframe(uint64_t stamp_ns) {
    const auto first_odom = interpolateOdom(odom_, stamp_ns);
    const Rot3 rot0 = first_odom.pose.value().rotation();
    const Pose3 pose0(rot0, Point3::Zero());
    const Vector3 vel0 = rot0.rotate(first_odom.velocity.value());
    odom_origin_ = first_odom.pose.value().translation();

    graph::PriorFactor prior;
    prior.stamp        = core::Timestamp(stamp_ns);
    prior.pose         = core::BodyInEnu(pose0);
    prior.velocity     = core::EnuVelocity(vel0);
    prior.bias         = bias0_;
    prior.gps_position = core::EnuPosition(Point3::Zero());
    storage_.prior = prior;

    graph::KeyframeEstimate est;
    est.stamp    = core::Timestamp(stamp_ns);
    est.pose     = core::BodyInEnu(pose0);
    est.velocity = core::EnuVelocity(vel0);
    est.bias     = bias0_;
    storage_.estimates.push_back(est);

    idx_ = 0;
}

// ─── GNSS processing ─────────────────────────────────────────────────────────

void MeasurementProcessor::addGnss(const domain::GnssFix& fix) {
    const uint64_t t_ns = fix.stamp.value();
    const auto& lla = fix.lla.value();

    if (idx_ == -1) {
        lat0_ = lla(0); lon0_ = lla(1); alt0_ = lla(2);
        initializeFirstKeyframe(t_ns);
        return;
    }

    createKeyframe(t_ns);

    Point3 enu = llaToEnu(lla(0), lla(1), lla(2), lat0_, lon0_, alt0_);
    bool corrupted = coin_(rng_);
    if (corrupted) {
        enu += Point3(spike_dir_(rng_) * cfg_.spike_metres,
                      spike_dir_(rng_) * cfg_.spike_metres,
                      spike_dir_(rng_) * cfg_.spike_metres);
        ++storage_.num_corrupted;
    }

    graph::StoredGpsFactor gps;
    gps.stamp        = core::Timestamp(t_ns);
    gps.position_enu = core::EnuPosition(enu);
    gps.corrupted    = corrupted;
    storage_.gps_factors.push_back(gps);
}

// ─── LiDAR processing ────────────────────────────────────────────────────────

void MeasurementProcessor::addLidar(core::Timestamp stamp,
                                    std::optional<Pose3> rel_pose) {
    if (idx_ == -1) return;

    createKeyframe(stamp.value());

    if (rel_pose && lidar_prev_idx_ >= 0) {
        graph::StoredLidarFactor lf;
        lf.stamp_from    = storage_.estimates[lidar_prev_idx_].stamp;
        lf.stamp_to      = core::Timestamp(stamp.value());
        lf.relative_pose = core::LidarDelta(*rel_pose);
        storage_.lidar_factors.push_back(lf);
    }
    lidar_prev_idx_ = idx_;
}

// ─── Keyframe creation ───────────────────────────────────────────────────────

void MeasurementProcessor::storeImuFactor(uint64_t stamp_ns) {
    graph::StoredImuFactor imu_f;
    imu_f.stamp_from = storage_.estimates[idx_ - 1].stamp;
    imu_f.stamp_to   = core::Timestamp(stamp_ns);
    imu_f.pim        = pim_;
    storage_.imu_factors.push_back(std::move(imu_f));
}

void MeasurementProcessor::storeKeyframeEstimate(uint64_t stamp_ns) {
    const auto odom_interp = interpolateOdom(odom_, stamp_ns);
    const Pose3 odom_pose = odom_interp.pose.value();
    const Pose3 enu_pose(odom_pose.rotation(),
                         odom_pose.translation() - odom_origin_);
    const Vector3 enu_vel = odom_pose.rotation().rotate(
        odom_interp.velocity.value());

    graph::KeyframeEstimate est;
    est.stamp    = core::Timestamp(stamp_ns);
    est.pose     = core::BodyInEnu(enu_pose);
    est.velocity = core::EnuVelocity(enu_vel);
    est.bias     = bias0_;
    storage_.estimates.push_back(est);
}

void MeasurementProcessor::createKeyframe(uint64_t stamp_ns) {
    if (prev_imu_) {
        const double dt = static_cast<double>(
            stamp_ns - prev_imu_->stamp.value()) * 1e-9;
        if (dt > 0.0 && dt < 1.0) {
            pim_.integrateMeasurement(
                prev_imu_->linear_acceleration.value(),
                prev_imu_->angular_velocity.value(), dt);
        }
        prev_imu_ = std::nullopt;
    }

    if (pim_.deltaTij() < 1e-6) return;

    ++idx_;
    storeImuFactor(stamp_ns);
    storeKeyframeEstimate(stamp_ns);
    pim_.resetIntegrationAndSetBias(bias0_);
}

// ─── Finalize ────────────────────────────────────────────────────────────────

graph::FactorStorage MeasurementProcessor::finalize() && {
    return std::move(storage_);
}

}  // namespace nufuse::processing
