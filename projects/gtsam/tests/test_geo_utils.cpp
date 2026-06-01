/// @file tests/test_geo_utils.cpp
/// @brief Unit tests for geodetic utilities and odometry interpolation.

#include <cmath>
#include <gtest/gtest.h>
#include <gtsam/geometry/Pose3.h>

#include "processing/geo_utils.hpp"

using namespace nufuse;

TEST(GeoUtils, LlaToEnuAtOriginIsZero) {
  const auto enu = processing::llaToEnu(48.0, 11.0, 500.0, 48.0, 11.0, 500.0);
  EXPECT_NEAR(enu.x(), 0.0, 1e-10);
  EXPECT_NEAR(enu.y(), 0.0, 1e-10);
  EXPECT_NEAR(enu.z(), 0.0, 1e-10);
}

TEST(GeoUtils, LlaToEnuNorthward) {
  // 1 degree north at equator ≈ 111,195 m
  const auto enu = processing::llaToEnu(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  EXPECT_NEAR(enu.y(), 111195.0, 200.0);  // North component
  EXPECT_NEAR(enu.x(), 0.0, 1e-10);       // East component
  EXPECT_NEAR(enu.z(), 0.0, 1e-10);       // Up component
}

TEST(GeoUtils, LlaToEnuEastward) {
  // 1 degree east at equator ≈ 111,195 m
  const auto enu = processing::llaToEnu(0.0, 1.0, 0.0, 0.0, 0.0, 0.0);
  EXPECT_NEAR(enu.x(), 111195.0, 200.0);  // East component
  EXPECT_NEAR(enu.y(), 0.0, 1e-10);       // North component
}

TEST(GeoUtils, LlaToEnuAltitude) {
  const auto enu = processing::llaToEnu(48.0, 11.0, 510.0, 48.0, 11.0, 500.0);
  EXPECT_NEAR(enu.z(), 10.0, 1e-10);
}

TEST(GeoUtils, FindNearestOdomExact) {
  std::vector<domain::OdomMeasurement> odom(3);
  odom[0].stamp = core::Timestamp(100);
  odom[0].velocity = core::BodyVector(gtsam::Vector3(1.0, 0.0, 0.0));
  odom[1].stamp = core::Timestamp(200);
  odom[1].velocity = core::BodyVector(gtsam::Vector3(2.0, 0.0, 0.0));
  odom[2].stamp = core::Timestamp(300);
  odom[2].velocity = core::BodyVector(gtsam::Vector3(3.0, 0.0, 0.0));

  const auto& result = processing::findNearestOdom(odom, 200);
  EXPECT_EQ(result.stamp.value(), 200);
}

TEST(GeoUtils, FindNearestOdomBetween) {
  std::vector<domain::OdomMeasurement> odom(3);
  odom[0].stamp = core::Timestamp(100);
  odom[0].velocity = core::BodyVector(gtsam::Vector3(1.0, 0.0, 0.0));
  odom[1].stamp = core::Timestamp(200);
  odom[1].velocity = core::BodyVector(gtsam::Vector3(2.0, 0.0, 0.0));
  odom[2].stamp = core::Timestamp(300);
  odom[2].velocity = core::BodyVector(gtsam::Vector3(3.0, 0.0, 0.0));

  // 190 is closer to 200 than 100
  const auto& result = processing::findNearestOdom(odom, 190);
  EXPECT_EQ(result.stamp.value(), 200);
}

TEST(GeoUtils, FindNearestOdomBeyondEnd) {
  std::vector<domain::OdomMeasurement> odom(2);
  odom[0].stamp = core::Timestamp(100);
  odom[0].velocity = core::BodyVector(gtsam::Vector3(1.0, 0.0, 0.0));
  odom[1].stamp = core::Timestamp(200);
  odom[1].velocity = core::BodyVector(gtsam::Vector3(2.0, 0.0, 0.0));

  const auto& result = processing::findNearestOdom(odom, 999);
  EXPECT_EQ(result.stamp.value(), 200);
}
