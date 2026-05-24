/// @file processing/geo_utils.cpp
/// @brief Implementation of geodetic and interpolation utilities.

#include "processing/geo_utils.hpp"

#include <algorithm>
#include <cmath>

namespace nufuse::processing {

using namespace gtsam;

gtsam::Point3 llaToEnu(double lat, double lon, double alt,
                        double lat0, double lon0, double alt0) {
    constexpr double kDeg2Rad = M_PI / 180.0;
    constexpr double kR = 6'378'137.0;
    return Point3(
        (lon - lon0) * kDeg2Rad * kR * std::cos(lat0 * kDeg2Rad),
        (lat - lat0) * kDeg2Rad * kR,
        alt - alt0);
}

domain::OdomMeasurement interpolateOdom(
    const std::vector<domain::OdomMeasurement>& odom, uint64_t stamp_ns) {
    auto it = std::lower_bound(odom.cbegin(), odom.cend(), stamp_ns,
        [](const domain::OdomMeasurement& m, uint64_t t) {
            return m.stamp.value() < t;
        });

    if (it == odom.cend())   return odom.back();
    if (it == odom.cbegin()) return odom.front();

    const auto& hi = *it;
    const auto& lo = *std::prev(it);

    const double alpha = static_cast<double>(stamp_ns - lo.stamp.value()) /
                         static_cast<double>(hi.stamp.value() - lo.stamp.value());

    const Pose3 pose0 = lo.pose.value();
    const Pose3 pose1 = hi.pose.value();
    const Pose3 pose_interp = pose0.compose(
        Pose3::Expmap(alpha * Pose3::Logmap(pose0.between(pose1))));

    const Vector3 vel = (1.0 - alpha) * lo.velocity.value()
                      +        alpha  * hi.velocity.value();
    const Vector3 acc = (1.0 - alpha) * lo.acceleration.value()
                      +        alpha  * hi.acceleration.value();
    const Vector3 angvel = (1.0 - alpha) * lo.angular_velocity.value()
                         +        alpha  * hi.angular_velocity.value();

    domain::OdomMeasurement out;
    out.stamp            = core::Timestamp(stamp_ns);
    out.pose             = core::BodyInMap(pose_interp);
    out.velocity         = core::BodyVector(vel);
    out.acceleration     = core::BodyVector(acc);
    out.angular_velocity = core::BodyVector(angvel);
    return out;
}

}  // namespace nufuse::processing
