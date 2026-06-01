/// @file processing/measurement_processor.cpp
/// @brief Implementation of MeasurementProcessor.

#include "processing/measurement_processor.hpp"

#include <cmath>

#include "processing/geo_utils.hpp"

namespace nufuse::processing {

using namespace gtsam;

// ─── IMU parameters ──────────────────────────────────────────────────────────

static auto makeImuParams(const core::ImuIntrinsicsConfig& imu) {
  auto params = PreintegratedCombinedMeasurements::Params::MakeSharedU(imu.gravity);
  params->accelerometerCovariance = I_3x3 * std::pow(imu.accel_noise_density, 2);
  params->gyroscopeCovariance = I_3x3 * std::pow(imu.gyro_noise_density, 2);
  params->integrationCovariance = I_3x3 * std::pow(imu.integration_sigma, 2);
  params->biasAccCovariance = I_3x3 * std::pow(imu.bias_accel_sigma, 2);
  params->biasOmegaCovariance = I_3x3 * std::pow(imu.bias_gyro_sigma, 2);
  return params;
}

// ─── Construction ────────────────────────────────────────────────────────────

MeasurementProcessor::MeasurementProcessor(const ProcessorConfig& cfg,
                                           const std::vector<domain::OdomMeasurement>& odom)
    : cfg_(cfg),
      odom_(odom),
      bias0_(cfg.initial_bias),
      pim_(makeImuParams(cfg.imu), cfg.initial_bias),
      keyframe_interval_ns_(static_cast<uint64_t>(cfg.keyframe_interval_s * 1e9)) {}

// ─── IMU integration ─────────────────────────────────────────────────────────

void MeasurementProcessor::addImu(const domain::ImuMeasurement& measurement) {
  if (prev_imu_) {
    const double dt =
        static_cast<double>(measurement.stamp.value() - prev_imu_->stamp.value()) * 1e-9;
    if (dt > 0.0 && dt < 1.0) {
      pim_.integrateMeasurement(prev_imu_->linear_acceleration.value(),
                                prev_imu_->angular_velocity.value(), dt);
    }
  }
  prev_imu_ = measurement;

  // If graph hasn't been initialized (by GPS), do it now on first IMU
  if (idx_ == -1) {
    initializeFirstKeyframe(measurement.stamp.value());
    last_imu_keyframe_stamp_ns_ = measurement.stamp.value();
    return;
  }

  // For IMU-only operation: create keyframes at regular intervals
  if (measurement.stamp.value() - last_imu_keyframe_stamp_ns_ >= keyframe_interval_ns_) {
    createKeyframe(measurement.stamp.value());
    last_imu_keyframe_stamp_ns_ = measurement.stamp.value();
  }
}

// ─── First keyframe initialization ──────────────────────────────────────────

void MeasurementProcessor::initializeFirstKeyframe(uint64_t stamp_ns) {
  const auto first_odom = interpolateOdom(odom_, stamp_ns);
  const Rot3 initial_rotation = first_odom.pose.value().rotation();
  // If init_from_gt is true, use ground truth translation; otherwise origin is (0,0,0)
  const Point3 initial_position =
      cfg_.init_from_gt ? first_odom.pose.value().translation() : Point3::Zero();
  const Pose3 initial_pose(initial_rotation, initial_position);
  const Vector3 initial_velocity = initial_rotation.rotate(first_odom.velocity.value());
  odom_origin_ = first_odom.pose.value().translation();

  graph::PriorFactor prior;
  prior.stamp = core::Timestamp(stamp_ns);
  prior.pose = core::BodyInEnu(initial_pose);
  prior.velocity = core::EnuVelocity(initial_velocity);
  prior.bias = bias0_;
  prior.gps_position = core::EnuPosition(cfg_.init_from_gt ? initial_position : Point3::Zero());
  storage_.prior = prior;

  graph::KeyframeEstimate estimate;
  estimate.stamp = core::Timestamp(stamp_ns);
  estimate.pose = core::BodyInEnu(initial_pose);
  estimate.velocity = core::EnuVelocity(initial_velocity);
  estimate.bias = bias0_;
  storage_.estimates.push_back(estimate);

  idx_ = 0;
}

// ─── GNSS processing ─────────────────────────────────────────────────────────

void MeasurementProcessor::addGnss(const domain::GnssFix& fix, bool corrupted) {
  const uint64_t stamp_ns = fix.stamp.value();
  const auto& lla = fix.lla.value();

  if (idx_ == -1) {
    lat0_ = lla(0);
    lon0_ = lla(1);
    alt0_ = lla(2);
    initializeFirstKeyframe(stamp_ns);
    return;
  }

  createKeyframe(stamp_ns);

  Point3 enu = llaToEnu(lla(0), lla(1), lla(2), lat0_, lon0_, alt0_);
  last_imu_keyframe_stamp_ns_ = stamp_ns;

  if (corrupted) {
    ++storage_.num_corrupted;
  }

  graph::StoredGpsFactor gps_factor;
  gps_factor.stamp = core::Timestamp(stamp_ns);
  gps_factor.position_enu = core::EnuPosition(enu);
  gps_factor.corrupted = corrupted;
  storage_.gps_factors.push_back(gps_factor);
}

// ─── LiDAR processing ────────────────────────────────────────────────────────

void MeasurementProcessor::addLidar(core::Timestamp stamp, std::optional<Pose3> rel_pose) {
  if (idx_ == -1) return;

  createKeyframe(stamp.value());
  last_imu_keyframe_stamp_ns_ = stamp.value();  // Reset IMU interval timer

  if (rel_pose && lidar_prev_idx_ >= 0) {
    graph::StoredLidarFactor lidar_factor;
    lidar_factor.stamp_from = storage_.estimates[lidar_prev_idx_].stamp;
    lidar_factor.stamp_to = core::Timestamp(stamp.value());
    lidar_factor.relative_pose = core::LidarDelta(*rel_pose);
    storage_.lidar_factors.push_back(lidar_factor);
  }
  lidar_prev_idx_ = idx_;
}

// ─── Keyframe creation ───────────────────────────────────────────────────────

void MeasurementProcessor::storeImuFactor(uint64_t stamp_ns) {
  graph::StoredImuFactor imu_factor;
  imu_factor.stamp_from = storage_.estimates[idx_ - 1].stamp;
  imu_factor.stamp_to = core::Timestamp(stamp_ns);
  imu_factor.pim = pim_;
  storage_.imu_factors.push_back(std::move(imu_factor));
}

void MeasurementProcessor::storeKeyframeEstimate(uint64_t stamp_ns) {
  const auto interpolated_odom = interpolateOdom(odom_, stamp_ns);
  const Pose3 odom_pose = interpolated_odom.pose.value();
  const Pose3 enu_pose(odom_pose.rotation(), odom_pose.translation() - odom_origin_);
  const Vector3 enu_velocity = odom_pose.rotation().rotate(interpolated_odom.velocity.value());

  graph::KeyframeEstimate estimate;
  estimate.stamp = core::Timestamp(stamp_ns);
  estimate.pose = core::BodyInEnu(enu_pose);
  estimate.velocity = core::EnuVelocity(enu_velocity);
  estimate.bias = bias0_;
  storage_.estimates.push_back(estimate);
}

void MeasurementProcessor::createKeyframe(uint64_t stamp_ns) {
  if (prev_imu_) {
    const double dt = static_cast<double>(stamp_ns - prev_imu_->stamp.value()) * 1e-9;
    if (dt > 0.0 && dt < 1.0) {
      pim_.integrateMeasurement(prev_imu_->linear_acceleration.value(),
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
