/// @file core/gtsam_boundary.hpp
/// @brief Type-safe helpers for crossing the GTSAM optimization boundary.
///
/// GTSAM operates on raw gtsam::Pose3, Vector3, etc. These helpers make the
/// trust boundary explicit: typed data goes in, raw data crosses the boundary,
/// and typed data comes back out with documented frame assumptions.

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/Values.h>

#include "core/types.hpp"

namespace nufuse::core {

// ============================================================================
// Typed symbol keys — prevent inserting wrong value types
// ============================================================================

/// @brief Typed wrapper around a GTSAM symbol for compile-time variable semantics.
template <typename ValueType>
struct TypedSymbol {
  gtsam::Symbol symbol;

  explicit TypedSymbol(char c, std::uint64_t j) : symbol(c, j) {}
  explicit TypedSymbol(gtsam::Symbol s) : symbol(s) {}

  [[nodiscard]] gtsam::Key key() const { return symbol; }
  [[nodiscard]] std::uint64_t index() const { return symbol.index(); }
  operator gtsam::Key() const { return symbol; }  // NOLINT: implicit for GTSAM API compat
};

/// Pose symbol X(i) — holds gtsam::Pose3 representing BodyInEnu.
/// NOTE: gtsam::symbol_shorthand::X uses lowercase 'x'. Use poseSymbol() helpers.
using PoseSymbol = TypedSymbol<gtsam::Pose3>;

/// Velocity symbol V(i) — holds gtsam::Vector3 representing EnuVelocity.
/// NOTE: gtsam::symbol_shorthand::V uses lowercase 'v'. Use velocitySymbol() helpers.
using VelocitySymbol = TypedSymbol<gtsam::Vector3>;

/// Bias symbol B(i) — holds gtsam::imuBias::ConstantBias.
/// NOTE: gtsam::symbol_shorthand::B uses lowercase 'b'. Use biasSymbol() helpers.
using BiasSymbol = TypedSymbol<gtsam::imuBias::ConstantBias>;

/// Lidar extrinsic symbol L(0) — holds gtsam::Pose3 representing BodyFromLidar.
/// NOTE: Builder uses explicit Symbol('L', 0) — uppercase 'L'.
using LidarExtrinsicSymbol = TypedSymbol<gtsam::Pose3>;

/// Radar extrinsic symbol R(i) — holds gtsam::Pose3 representing BodyFromRadar.
/// NOTE: Builder uses explicit Symbol('R', i) — uppercase 'R'.
using RadarExtrinsicSymbol = TypedSymbol<gtsam::Pose3>;

/// Odometry scale symbol S(0) — holds double.
using OdomScaleSymbol = TypedSymbol<double>;

// ── Symbol factory functions matching gtsam::symbol_shorthand convention ─────

/// @brief Create pose symbol matching gtsam::symbol_shorthand::X (lowercase 'x').
[[nodiscard]] inline PoseSymbol poseSymbol(int i) {
  return PoseSymbol('x', static_cast<std::uint64_t>(i));
}

/// @brief Create velocity symbol matching gtsam::symbol_shorthand::V (lowercase 'v').
[[nodiscard]] inline VelocitySymbol velocitySymbol(int i) {
  return VelocitySymbol('v', static_cast<std::uint64_t>(i));
}

/// @brief Create bias symbol matching gtsam::symbol_shorthand::B (lowercase 'b').
[[nodiscard]] inline BiasSymbol biasSymbol(int i) {
  return BiasSymbol('b', static_cast<std::uint64_t>(i));
}

// ============================================================================
// Insertion helpers — typed data → GTSAM Values
// ============================================================================

/// @brief Insert a BodyInEnu pose into GTSAM Values at the given pose symbol.
inline void insertPose(gtsam::Values& values, PoseSymbol sym, const BodyInEnu& pose) {
  values.insert(sym.key(), pose.value());
}

/// @brief Insert an EnuVelocity into GTSAM Values at the given velocity symbol.
inline void insertVelocity(gtsam::Values& values, VelocitySymbol sym, const EnuVelocity& vel) {
  values.insert(sym.key(), vel.value());
}

/// @brief Insert a BodyFromLidar extrinsic into GTSAM Values.
inline void insertLidarExtrinsic(gtsam::Values& values, LidarExtrinsicSymbol sym,
                                 const BodyFromLidar& extrinsic) {
  values.insert(sym.key(), extrinsic.value());
}

// ============================================================================
// Extraction helpers — GTSAM Values → typed results
// ============================================================================

/// @brief Extract BodyInEnu from optimized GTSAM Values.
/// TRUST BOUNDARY: Assumes the optimizer preserved ENU frame semantics for X(i).
[[nodiscard]] inline BodyInEnu extractPose(const gtsam::Values& values, PoseSymbol sym) {
  return BodyInEnu(values.at<gtsam::Pose3>(sym.key()));
}

/// @brief Extract EnuVelocity from optimized GTSAM Values.
/// TRUST BOUNDARY: Assumes the optimizer preserved ENU frame semantics for V(i).
[[nodiscard]] inline EnuVelocity extractVelocity(const gtsam::Values& values, VelocitySymbol sym) {
  return EnuVelocity(values.at<gtsam::Vector3>(sym.key()));
}

/// @brief Extract BodyFromLidar from optimized GTSAM Values.
/// TRUST BOUNDARY: Assumes L(0) represents Body←LidarTop transform.
[[nodiscard]] inline BodyFromLidar extractLidarExtrinsic(const gtsam::Values& values,
                                                         LidarExtrinsicSymbol sym) {
  return BodyFromLidar(values.at<gtsam::Pose3>(sym.key()));
}

/// @brief Extract a radar extrinsic (BodyFromRadar*) from optimized GTSAM Values.
/// TRUST BOUNDARY: Caller must know which radar frame R(i) corresponds to.
[[nodiscard]] inline gtsam::Pose3 extractRadarExtrinsicRaw(const gtsam::Values& values,
                                                           RadarExtrinsicSymbol sym) {
  return values.at<gtsam::Pose3>(sym.key());
}

// ============================================================================
// Angular velocity in body frame — semantic wrapper for factor inputs
// ============================================================================

/// @brief Angular velocity measured/interpolated in the body frame [rad/s].
/// Used as a known quantity (not optimized) in NHC, radar, and forward-velocity factors.
using AngularVelocityBody = slam::geometry::TaggedPoint<gtsam::Vector3, cf::Body>;

// ============================================================================
// Radar bearing in radar frame — semantic wrapper
// ============================================================================

/// @brief Unit bearing vector in a radar sensor frame (z-squashed, normalized).
/// The specific radar frame depends on context (RadarFront, RadarFrontLeft, etc.).
using RadarBearing = slam::geometry::TaggedPoint<gtsam::Vector3, cf::RadarFront>;

// ============================================================================
// Radar sensor identification
// ============================================================================

/// @brief Identifies a radar sensor by index (0=Front, 1=FrontLeft, ..., 4=BackRight).
enum class RadarSensorId : int {
  Front = 0,
  FrontLeft = 1,
  FrontRight = 2,
  BackLeft = 3,
  BackRight = 4,
};

/// @brief Create a radar extrinsic symbol R(i) from a RadarSensorId.
[[nodiscard]] inline RadarExtrinsicSymbol radarSymbol(RadarSensorId id) {
  return RadarExtrinsicSymbol('R', static_cast<std::uint64_t>(id));
}

// ============================================================================
// Odometry scale — strong type for the scalar correction
// ============================================================================

/// @brief Odometry scale correction: actual_speed = measured_speed * (1 + delta).
struct OdomScaleDelta {
  double value = 0.0;
  explicit OdomScaleDelta(double v = 0.0) : value(v) {}
};

}  // namespace nufuse::core
