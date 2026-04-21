/// @file nuscenes_mcap_loader.cpp
/// @brief Implementation of the NuScenes MCAP loader.

#define MCAP_IMPLEMENTATION
#include <mcap/reader.hpp>

#include "nuscenes_mcap_loader.hpp"

#include "foxglove/FrameTransform.pb.h"
#include "foxglove/LocationFix.pb.h"

#include <nlohmann/json.hpp>

#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {

/// @brief Converts MCAP log-time (nanoseconds) to a TimestampId.
TimestampId toStampId(uint64_t log_time_ns) {
    return TimestampId(log_time_ns);
}

/// @brief Checks whether @p topic starts with @p prefix.
bool startsWith(std::string_view topic, std::string_view prefix) {
    return topic.size() >= prefix.size() &&
           topic.substr(0, prefix.size()) == prefix;
}

/// @brief Returns true for radar topics (/RADAR_*).
bool isRadarTopic(std::string_view topic) {
    return startsWith(topic, "/RADAR_");
}

/// @brief Returns true for compressed-image topics.
bool isImageTopic(std::string_view topic) {
    return topic.find("/image_rect_compressed") != std::string_view::npos;
}

/// @brief Builds a gtsam::Pose3 from a foxglove::FrameTransform message.
gtsam::Pose3 toPose3(const foxglove::FrameTransform& tf) {
    gtsam::Rot3 rot = gtsam::Rot3::Quaternion(
        tf.rotation().w(), tf.rotation().x(),
        tf.rotation().y(), tf.rotation().z());
    gtsam::Point3 pos(tf.translation().x(), tf.translation().y(),
                      tf.translation().z());
    return gtsam::Pose3(rot, pos);
}

/// @brief Assigns a FrameTransform to the matching typed field in calibration.
///
/// @return true if the child_frame_id matched a known sensor, false otherwise.
bool assignExtrinsic(ExtrinsicCalibration& cal,
                     const foxglove::FrameTransform& tf) {
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

}  // anonymous namespace

namespace nuscenes {

SceneData loadMcap(const std::filesystem::path& mcap_path) {
    SceneData scene;

    mcap::McapReader reader;
    auto status = reader.open(mcap_path.string());
    if (!status.ok()) {
        throw std::runtime_error("Failed to open MCAP: " + mcap_path.string() +
                                 " (" + status.message + ")");
    }

    status = reader.readSummary(mcap::ReadSummaryMethod::NoFallbackScan);
    if (!status.ok()) {
        status = reader.readSummary(mcap::ReadSummaryMethod::AllowFallbackScan);
    }

    for (const auto& view : reader.readMessages()) {
        const auto& topic = view.channel->topic;
        const auto* data = reinterpret_cast<const char*>(view.message.data);
        const auto size = static_cast<int>(view.message.dataSize);
        const auto stamp = toStampId(view.message.logTime);

        // ---- JSON-encoded: /imu -------------------------------------------
        if (topic == "/imu") {
            const auto j = nlohmann::json::parse(data, data + size, nullptr, false);
            if (j.is_discarded()) continue;

            ImuMeasurement m;
            m.stamp = stamp;
            const auto& a = j["linear_accel"];
            m.linear_acceleration = slam::geometry::TaggedPoint<gtsam::Vector3, ImuFrame>(
                gtsam::Vector3(a["x"].get<double>(), a["y"].get<double>(), a["z"].get<double>()));
            const auto& g = j["rotation_rate"];
            m.angular_velocity = slam::geometry::TaggedPoint<gtsam::Vector3, ImuFrame>(
                gtsam::Vector3(g["x"].get<double>(), g["y"].get<double>(), g["z"].get<double>()));
            const auto& q = j["q"];
            m.orientation = gtsam::Rot3::Quaternion(q["w"].get<double>(), q["x"].get<double>(),
                                                    q["y"].get<double>(), q["z"].get<double>());
            scene.imu.push_back(std::move(m));
            continue;
        }

        // ---- JSON-encoded: /odom ------------------------------------------
        if (topic == "/odom") {
            const auto j = nlohmann::json::parse(data, data + size, nullptr, false);
            if (j.is_discarded()) continue;

            OdomMeasurement m;
            m.stamp = stamp;

            const auto& p = j["pos"];
            const auto& o = j["orientation"];
            gtsam::Rot3 rot = gtsam::Rot3::Quaternion(
                o["w"].get<double>(), o["x"].get<double>(),
                o["y"].get<double>(), o["z"].get<double>());
            gtsam::Point3 pos(p["x"].get<double>(), p["y"].get<double>(),
                              p["z"].get<double>());
            m.pose = slam::geometry::Pose<gtsam::Pose3, BodyFrame, MapFrame>(
                gtsam::Pose3(rot, pos));

            const auto& v = j["vel"];
            m.velocity = slam::geometry::TaggedPoint<gtsam::Vector3, BodyFrame>(
                gtsam::Vector3(v["x"].get<double>(), v["y"].get<double>(), v["z"].get<double>()));
            const auto& ac = j["accel"];
            m.acceleration = slam::geometry::TaggedPoint<gtsam::Vector3, BodyFrame>(
                gtsam::Vector3(ac["x"].get<double>(), ac["y"].get<double>(), ac["z"].get<double>()));
            const auto& gr = j["rotation_rate"];
            m.angular_velocity = slam::geometry::TaggedPoint<gtsam::Vector3, BodyFrame>(
                gtsam::Vector3(gr["x"].get<double>(), gr["y"].get<double>(), gr["z"].get<double>()));
            scene.odom.push_back(std::move(m));
            continue;
        }

        // ---- Protobuf: /gps (foxglove.LocationFix) ------------------------
        if (topic == "/gps") {
            foxglove::LocationFix fix;
            if (fix.ParseFromArray(data, size)) {
                GnssFix m;
                m.stamp = stamp;
                m.lla = slam::geometry::TaggedPoint<gtsam::Point3, Wgs84Frame>(
                    gtsam::Point3(fix.latitude(), fix.longitude(), fix.altitude()));
                m.position_covariance.assign(fix.position_covariance().begin(),
                                             fix.position_covariance().end());
                scene.gnss.push_back(std::move(m));
            }
            continue;
        }

        // ---- Protobuf: /LIDAR_TOP, /RADAR_* (foxglove.PointCloud) ---------
        if (topic == "/LIDAR_TOP" || isRadarTopic(topic)) {
            foxglove::PointCloud msg;
            if (msg.ParseFromArray(data, size)) {
                auto& dest = (topic == "/LIDAR_TOP") ? scene.lidar : scene.radar;
                dest.push_back({stamp, std::move(msg)});
            }
            continue;
        }

        // ---- Protobuf: /CAM_*/image_rect_compressed -----------------------
        if (isImageTopic(topic)) {
            foxglove::CompressedImage msg;
            if (msg.ParseFromArray(data, size)) {
                scene.images.push_back({stamp, std::move(msg)});
            }
            continue;
        }

        // ---- Protobuf: /tf (foxglove.FrameTransform) ----------------------
        if (topic == "/tf") {
            foxglove::FrameTransform msg;
            if (msg.ParseFromArray(data, size)) {
                if (msg.parent_frame_id() == "base_link") {
                    // Static sensor extrinsic (base_link → sensor).
                    assignExtrinsic(scene.extrinsics, msg);
                } else if (msg.parent_frame_id() == "map" &&
                           msg.child_frame_id() == "base_link") {
                    // Dynamic ego pose (map → base_link).
                    using EgoTransform =
                        slam::geometry::Transform<gtsam::Pose3, MapFrame, BodyFrame>;
                    Stamped<EgoTransform> ego;
                    ego.stamp = stamp;
                    ego.msg = EgoTransform(toPose3(msg));
                    scene.ego_poses.push_back(std::move(ego));
                }
            }
            continue;
        }
    }

    reader.close();
    return scene;
}

}  // namespace nuscenes
