/// @file processing/geo_utils.cpp
/// @brief Implementation of geodetic and interpolation utilities.

#include "processing/geo_utils.hpp"

#include <algorithm>
#include <cmath>

namespace nufuse::processing {

using namespace gtsam;

gtsam::Point3 llaToEnu(double lat, double lon, double alt, double lat0, double lon0, double alt0) {
  constexpr double kDeg2Rad = M_PI / 180.0;
  constexpr double kR = 6'378'137.0;
  return Point3((lon - lon0) * kDeg2Rad * kR * std::cos(lat0 * kDeg2Rad),
                (lat - lat0) * kDeg2Rad * kR, alt - alt0);
}

domain::OdomMeasurement interpolateOdom(const std::vector<domain::OdomMeasurement>& odom,
                                        uint64_t stamp_ns) {
  auto it = std::lower_bound(odom.cbegin(), odom.cend(), stamp_ns,
                             [](const domain::OdomMeasurement& measurement, uint64_t target_stamp) {
                               return measurement.stamp.value() < target_stamp;
                             });

  if (it == odom.cend()) return odom.back();
  if (it == odom.cbegin()) return odom.front();

  const auto& upper = *it;
  const auto& lower = *std::prev(it);

  const double alpha = static_cast<double>(stamp_ns - lower.stamp.value()) /
                       static_cast<double>(upper.stamp.value() - lower.stamp.value());

  const Pose3 pose_lower = lower.pose.value();
  const Pose3 pose_upper = upper.pose.value();
  const Pose3 pose_interp =
      pose_lower.compose(Pose3::Expmap(alpha * Pose3::Logmap(pose_lower.between(pose_upper))));

  const Vector3 velocity_interp =
      (1.0 - alpha) * lower.velocity.value() + alpha * upper.velocity.value();
  const Vector3 acceleration_interp =
      (1.0 - alpha) * lower.acceleration.value() + alpha * upper.acceleration.value();
  const Vector3 angular_velocity_interp =
      (1.0 - alpha) * lower.angular_velocity.value() + alpha * upper.angular_velocity.value();

  domain::OdomMeasurement result;
  result.stamp = core::Timestamp(stamp_ns);
  result.pose = core::BodyInMap(pose_interp);
  result.velocity = core::BodyVector(velocity_interp);
  result.acceleration = core::BodyVector(acceleration_interp);
  result.angular_velocity = core::BodyVector(angular_velocity_interp);
  return result;
}

const domain::OdomMeasurement& findNearestOdom(const std::vector<domain::OdomMeasurement>& odom,
                                               uint64_t stamp_ns) {
  auto it = std::lower_bound(
      odom.begin(), odom.end(), stamp_ns,
      [](const domain::OdomMeasurement& m, uint64_t s) { return m.stamp.value() < s; });
  if (it == odom.end())
    --it;
  else if (it != odom.begin() &&
           (it->stamp.value() - stamp_ns) > (stamp_ns - std::prev(it)->stamp.value()))
    --it;
  return *it;
}

}  // namespace nufuse::processing
