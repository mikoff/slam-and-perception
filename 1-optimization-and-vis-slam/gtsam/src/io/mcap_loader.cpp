/// @file io/mcap_loader.cpp
/// @brief Implementation of the MCAP loader.

#define MCAP_IMPLEMENTATION
#include "io/mcap_loader.hpp"

#include <iostream>
#include <mcap/reader.hpp>
#include <mcap/writer.hpp>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <string_view>

#include "foxglove/FrameTransform.pb.h"
#include "foxglove/LocationFix.pb.h"

namespace nufuse::io {
namespace {

core::Timestamp toTimestamp(uint64_t ns) {
  return core::Timestamp(ns);
}

bool startsWith(std::string_view s, std::string_view prefix) {
  return s.size() >= prefix.size() && s.substr(0, prefix.size()) == prefix;
}

bool isRadarTopic(std::string_view topic) {
  return startsWith(topic, "/RADAR_");
}

bool isImageTopic(std::string_view topic) {
  return topic.find("/image_rect_compressed") != std::string_view::npos;
}

gtsam::Pose3 toPose3(const foxglove::FrameTransform& tf) {
  gtsam::Rot3 rot = gtsam::Rot3::Quaternion(tf.rotation().w(), tf.rotation().x(), tf.rotation().y(),
                                            tf.rotation().z());
  gtsam::Point3 pos(tf.translation().x(), tf.translation().y(), tf.translation().z());
  return gtsam::Pose3(rot, pos);
}

bool assignExtrinsic(domain::ExtrinsicCalibration& cal, const foxglove::FrameTransform& tf) {
  const std::string& child = tf.child_frame_id();
  const auto pose = toPose3(tf);

  if (child == "LIDAR_TOP") {
    cal.body_from_lidar_top.emplace(pose);
  } else if (child == "CAM_FRONT") {
    cal.body_from_cam_front.emplace(pose);
  } else if (child == "CAM_FRONT_LEFT") {
    cal.body_from_cam_front_left.emplace(pose);
  } else if (child == "CAM_FRONT_RIGHT") {
    cal.body_from_cam_front_right.emplace(pose);
  } else if (child == "CAM_BACK") {
    cal.body_from_cam_back.emplace(pose);
  } else if (child == "CAM_BACK_LEFT") {
    cal.body_from_cam_back_left.emplace(pose);
  } else if (child == "CAM_BACK_RIGHT") {
    cal.body_from_cam_back_right.emplace(pose);
  } else if (child == "RADAR_FRONT") {
    cal.body_from_radar_front.emplace(pose);
  } else if (child == "RADAR_FRONT_LEFT") {
    cal.body_from_radar_front_left.emplace(pose);
  } else if (child == "RADAR_FRONT_RIGHT") {
    cal.body_from_radar_front_right.emplace(pose);
  } else if (child == "RADAR_BACK_LEFT") {
    cal.body_from_radar_back_left.emplace(pose);
  } else if (child == "RADAR_BACK_RIGHT") {
    cal.body_from_radar_back_right.emplace(pose);
  } else {
    return false;
  }
  return true;
}

void parseImu(domain::SceneData& scene, const char* data, int size, core::Timestamp stamp) {
  const auto json = nlohmann::json::parse(data, data + size, nullptr, false);
  if (json.is_discarded()) return;

  domain::ImuMeasurement measurement;
  measurement.stamp = stamp;

  const auto& accel = json["linear_accel"];
  measurement.linear_acceleration = core::ImuVector(
      gtsam::Vector3(accel["x"].get<double>(), accel["y"].get<double>(), accel["z"].get<double>()));

  const auto& gyro = json["rotation_rate"];
  measurement.angular_velocity = core::ImuVector(
      gtsam::Vector3(gyro["x"].get<double>(), gyro["y"].get<double>(), gyro["z"].get<double>()));

  const auto& quat = json["q"];
  measurement.orientation =
      gtsam::Rot3::Quaternion(quat["w"].get<double>(), quat["x"].get<double>(),
                              quat["y"].get<double>(), quat["z"].get<double>());

  scene.imu.push_back(std::move(measurement));
}

void parseOdom(domain::SceneData& scene, const char* data, int size, core::Timestamp stamp) {
  const auto json = nlohmann::json::parse(data, data + size, nullptr, false);
  if (json.is_discarded()) return;

  domain::OdomMeasurement measurement;
  measurement.stamp = stamp;

  const auto& position = json["pos"];
  const auto& orientation = json["orientation"];
  gtsam::Rot3 rot =
      gtsam::Rot3::Quaternion(orientation["w"].get<double>(), orientation["x"].get<double>(),
                              orientation["y"].get<double>(), orientation["z"].get<double>());
  gtsam::Point3 pos(position["x"].get<double>(), position["y"].get<double>(),
                    position["z"].get<double>());
  measurement.pose = core::BodyInMap(gtsam::Pose3(rot, pos));

  const auto& vel = json["vel"];
  measurement.velocity = core::BodyVector(
      gtsam::Vector3(vel["x"].get<double>(), vel["y"].get<double>(), vel["z"].get<double>()));

  const auto& accel = json["accel"];
  measurement.acceleration = core::BodyVector(
      gtsam::Vector3(accel["x"].get<double>(), accel["y"].get<double>(), accel["z"].get<double>()));

  const auto& ang_vel = json["rotation_rate"];
  measurement.angular_velocity = core::BodyVector(gtsam::Vector3(
      ang_vel["x"].get<double>(), ang_vel["y"].get<double>(), ang_vel["z"].get<double>()));

  scene.odom.push_back(std::move(measurement));
}

void parseGps(domain::SceneData& scene, const char* data, int size, core::Timestamp stamp) {
  foxglove::LocationFix fix;
  if (!fix.ParseFromArray(data, size)) return;

  domain::GnssFix measurement;
  measurement.stamp = stamp;
  measurement.lla = core::Wgs84Lla(gtsam::Point3(fix.latitude(), fix.longitude(), fix.altitude()));
  measurement.position_covariance.assign(fix.position_covariance().begin(),
                                         fix.position_covariance().end());
  scene.gnss.push_back(std::move(measurement));
}

void parseTf(domain::SceneData& scene, const char* data, int size, core::Timestamp stamp) {
  foxglove::FrameTransform transform;
  if (!transform.ParseFromArray(data, size)) return;

  if (transform.parent_frame_id() == "base_link") {
    assignExtrinsic(scene.extrinsics, transform);
  } else if (transform.parent_frame_id() == "map" && transform.child_frame_id() == "base_link") {
    core::Stamped<core::MapFromBody> ego_pose;
    ego_pose.stamp = stamp;
    ego_pose.data = core::MapFromBody(toPose3(transform));
    scene.ego_poses.push_back(std::move(ego_pose));
  }
}

}  // anonymous namespace

domain::SceneData loadMcap(const std::filesystem::path& mcap_path) {
  domain::SceneData scene;

  mcap::McapReader reader;
  auto status = reader.open(mcap_path.string());
  if (!status.ok()) {
    throw std::runtime_error("Failed to open MCAP: " + mcap_path.string() + " (" + status.message +
                             ")");
  }

  status = reader.readSummary(mcap::ReadSummaryMethod::NoFallbackScan);
  if (!status.ok()) {
    status = reader.readSummary(mcap::ReadSummaryMethod::AllowFallbackScan);
  }

  for (const auto& view : reader.readMessages()) {
    const auto& topic = view.channel->topic;
    const auto* raw_data = reinterpret_cast<const char*>(view.message.data);
    const auto data_size = static_cast<int>(view.message.dataSize);
    const auto stamp = toTimestamp(view.message.logTime);

    if (topic == "/imu") {
      parseImu(scene, raw_data, data_size, stamp);
    } else if (topic == "/odom") {
      parseOdom(scene, raw_data, data_size, stamp);
    } else if (topic == "/gps") {
      parseGps(scene, raw_data, data_size, stamp);
    } else if (topic == "/LIDAR_TOP" || isRadarTopic(topic)) {
      foxglove::PointCloud point_cloud;
      if (point_cloud.ParseFromArray(raw_data, data_size)) {
        auto& dest = (topic == "/LIDAR_TOP") ? scene.lidar : scene.radar;
        dest.push_back({stamp, std::move(point_cloud)});
      }
    } else if (isImageTopic(topic)) {
      foxglove::CompressedImage image;
      if (image.ParseFromArray(raw_data, data_size)) {
        scene.images.push_back({stamp, std::move(image)});
      }
    } else if (topic == "/tf") {
      parseTf(scene, raw_data, data_size, stamp);
    }
  }

  reader.close();
  return scene;
}

}  // namespace nufuse::io
