/// @file processing/bias_estimator.cpp
/// @brief Implementation of initial IMU bias estimation.

#include "processing/bias_estimator.hpp"

#include <algorithm>

#include "processing/geo_utils.hpp"

namespace nufuse::processing {

gtsam::imuBias::ConstantBias estimateInitialBias(const std::vector<domain::ImuMeasurement>& imu,
                                                 const std::vector<domain::OdomMeasurement>& odom) {
  constexpr int kNumSamples = 100;
  const int n = std::min(kNumSamples, static_cast<int>(imu.size()));
  if (n == 0 || odom.empty()) return {};

  gtsam::Vector3 gyro_diff_sum = gtsam::Vector3::Zero();
  int count = 0;
  for (int i = 0; i < n; ++i) {
    const auto& nearest_odom = findNearestOdom(odom, imu[i].stamp.value());
    gyro_diff_sum += imu[i].angular_velocity.value() - nearest_odom.angular_velocity.value();
    ++count;
  }

  const gtsam::Vector3 gyro_bias = gyro_diff_sum / count;
  return gtsam::imuBias::ConstantBias(gtsam::Vector3::Zero(), gyro_bias);
}

}  // namespace nufuse::processing
