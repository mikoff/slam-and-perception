/// @file measurement_processor.cpp
/// @brief Implementation of MeasurementProcessor.

#include "measurement_processor.hpp"

#include <algorithm>
#include <cmath>
#include <random>

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

// ─── Coordinate conversion ───────────────────────────────────────────────────

Point3 llaToEnu(double lat, double lon, double alt,
                double lat0, double lon0, double alt0) {
    constexpr double kDeg2Rad = M_PI / 180.0;
    constexpr double kR       = 6'378'137.0;
    return Point3(
        (lon - lon0) * kDeg2Rad * kR * std::cos(lat0 * kDeg2Rad),
        (lat - lat0) * kDeg2Rad * kR,
        alt - alt0);
}

// ─── Odometry interpolation ──────────────────────────────────────────────────

OdomMeasurement interpolateOdom(const std::vector<OdomMeasurement>& odom,
                                uint64_t stamp_ns) {
    auto it = std::lower_bound(odom.cbegin(), odom.cend(), stamp_ns,
        [](const OdomMeasurement& m, uint64_t t) {
            return m.stamp.value() < t;
        });

    if (it == odom.cend())   return odom.back();
    if (it == odom.cbegin()) return odom.front();

    const OdomMeasurement& hi = *it;
    const OdomMeasurement& lo = *std::prev(it);

    const double alpha = static_cast<double>(stamp_ns - lo.stamp.value()) /
                         static_cast<double>(hi.stamp.value() - lo.stamp.value());

    const Pose3 pose0 = lo.pose.value();
    const Pose3 pose1 = hi.pose.value();
    const Pose3 pose_interp = pose0.compose(
        Pose3::Expmap(alpha * Pose3::Logmap(pose0.between(pose1))));

    const Vector3 vel_interp    = (1.0 - alpha) * lo.velocity.value()
                                +        alpha  * hi.velocity.value();
    const Vector3 acc_interp    = (1.0 - alpha) * lo.acceleration.value()
                                +        alpha  * hi.acceleration.value();
    const Vector3 angvel_interp = (1.0 - alpha) * lo.angular_velocity.value()
                                +        alpha  * hi.angular_velocity.value();

    OdomMeasurement out;
    out.stamp            = TimestampId(stamp_ns);
    out.pose             = decltype(out.pose)(pose_interp);
    out.velocity         = decltype(out.velocity)(vel_interp);
    out.acceleration     = decltype(out.acceleration)(acc_interp);
    out.angular_velocity = decltype(out.angular_velocity)(angvel_interp);
    return out;
}

// ─── MeasurementProcessor ────────────────────────────────────────────────────

MeasurementProcessor::MeasurementProcessor(
    const ProcessorConfig& cfg, const std::vector<OdomMeasurement>& odom)
    : cfg_(cfg)
    , odom_(odom)
    , pim_(makeImuParams(), imuBias::ConstantBias{})
    , rng_(cfg.rng_seed)
    , coin_(cfg.spike_fraction) {}

void MeasurementProcessor::addImu(const ImuMeasurement& m) {
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

void MeasurementProcessor::addGnss(const GnssFix& fix) {
    const uint64_t t_ns = fix.stamp.value();
    const auto& lla = fix.lla.value();

    if (idx_ == -1) {
        // First fix: establish ENU reference.
        lat0_ = lla(0);  lon0_ = lla(1);  alt0_ = lla(2);

        const auto first_odom = interpolateOdom(odom_, t_ns);
        const Rot3 rot0       = first_odom.pose.value().rotation();
        const Pose3 pose0(rot0, Point3::Zero());
        const Vector3 vel0    = rot0.rotate(first_odom.velocity.value());
        odom_origin_ = first_odom.pose.value().translation();

        using PoseType = slam::geometry::Pose<Pose3, BodyFrame, EnuFrame>;
        using VelType  = slam::geometry::TaggedPoint<Vector3, EnuFrame>;
        using PosType  = slam::geometry::TaggedPoint<Point3, EnuFrame>;

        PriorFactor prior;
        prior.keyframe    = KeyframeId(0);
        prior.pose        = PoseType(pose0);
        prior.velocity    = VelType(vel0);
        prior.bias        = bias0_;
        prior.gps_position = PosType(Point3::Zero());
        storage_.prior = prior;

        KeyframeEstimate est;
        est.id       = KeyframeId(0);
        est.pose     = PoseType(pose0);
        est.velocity = VelType(vel0);
        est.bias     = bias0_;
        storage_.estimates.push_back(est);

        idx_ = 0;
        return;
    }

    createKeyframe(t_ns);

    // GPS factor with optional corruption.
    Point3 enu = llaToEnu(lla(0), lla(1), lla(2), lat0_, lon0_, alt0_);
    bool corrupted = coin_(rng_);
    if (corrupted) {
        enu += Point3(spike_dir_(rng_) * cfg_.spike_metres,
                      spike_dir_(rng_) * cfg_.spike_metres,
                      spike_dir_(rng_) * cfg_.spike_metres);
        ++storage_.num_corrupted;
    }

    using PosType = slam::geometry::TaggedPoint<Point3, EnuFrame>;
    StoredGpsFactor gps;
    gps.keyframe     = KeyframeId(idx_);
    gps.position_enu = PosType(enu);
    gps.corrupted    = corrupted;
    storage_.gps_factors.push_back(gps);
}

void MeasurementProcessor::addLidar(uint64_t stamp_ns,
                                    std::optional<Pose3> rel_pose) {
    if (idx_ == -1) return;

    createKeyframe(stamp_ns);

    if (rel_pose && lidar_prev_idx_ >= 0) {
        using TfType = slam::geometry::Transform<gtsam::Pose3, LidarTopFrame, LidarTopFrame>;
        StoredLidarFactor lf;
        lf.from          = KeyframeId(lidar_prev_idx_);
        lf.to            = KeyframeId(idx_);
        lf.relative_pose = TfType(*rel_pose);
        storage_.lidar_factors.push_back(lf);
    }
    lidar_prev_idx_ = idx_;
}

FactorStorage MeasurementProcessor::finalize() && {
    return std::move(storage_);
}

void MeasurementProcessor::createKeyframe(uint64_t stamp_ns) {
    // Flush the last IMU sample up to this epoch boundary.
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

    // Skip creating a new keyframe if no IMU data was integrated since the
    // last one.  This avoids degenerate CombinedImuFactors with deltaTij ≈ 0
    // that produce NaN/inf cost (singular covariance).
    if (pim_.deltaTij() < 1e-6) {
        return;
    }

    ++idx_;

    // Store IMU factor.
    StoredImuFactor imu_f;
    imu_f.from = KeyframeId(idx_ - 1);
    imu_f.to   = KeyframeId(idx_);
    imu_f.pim  = pim_;
    storage_.imu_factors.push_back(std::move(imu_f));

    // Compute initial estimate from interpolated odometry.
    const auto odom_interp = interpolateOdom(odom_, stamp_ns);
    const Pose3 odom_pose  = odom_interp.pose.value();
    const Pose3 enu_pose(odom_pose.rotation(),
                         odom_pose.translation() - odom_origin_);
    const Vector3 enu_vel = odom_pose.rotation().rotate(
        odom_interp.velocity.value());

    using PoseType = slam::geometry::Pose<Pose3, BodyFrame, EnuFrame>;
    using VelType  = slam::geometry::TaggedPoint<Vector3, EnuFrame>;

    KeyframeEstimate est;
    est.id       = KeyframeId(idx_);
    est.pose     = PoseType(enu_pose);
    est.velocity = VelType(enu_vel);
    est.bias     = bias0_;
    storage_.estimates.push_back(est);

    pim_.resetIntegrationAndSetBias(bias0_);
}
