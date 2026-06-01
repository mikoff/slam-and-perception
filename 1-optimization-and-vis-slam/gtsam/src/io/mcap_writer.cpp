/// @file io/mcap_writer.cpp
/// @brief Writes optimization results to MCAP for Foxglove/Lichtblick visualization.

#include "io/mcap_writer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/descriptor.pb.h>
#include <gtsam/inference/Symbol.h>
#include <iostream>
#include <mcap/reader.hpp>
#include <mcap/writer.hpp>
#include <string>
#include <unordered_set>
#include <vector>

#include "foxglove/FrameTransform.pb.h"
#include "foxglove/LocationFix.pb.h"

namespace nufuse::io {
namespace {

// ─── Protobuf schema helpers ─────────────────────────────────────────────────

/// @brief Recursively collects all file descriptors needed for a message type.
void collectFileDescriptors(const google::protobuf::FileDescriptor* fd,
                            google::protobuf::FileDescriptorSet& fds,
                            std::unordered_set<std::string>& visited) {
  if (!fd || visited.count(fd->name())) return;
  visited.insert(fd->name());

  for (int i = 0; i < fd->dependency_count(); ++i) {
    collectFileDescriptors(fd->dependency(i), fds, visited);
  }
  fd->CopyTo(fds.add_file());
}

/// @brief Builds a serialized FileDescriptorSet for a protobuf message type.
std::string buildSchemaData(const google::protobuf::Descriptor* descriptor) {
  google::protobuf::FileDescriptorSet fds;
  std::unordered_set<std::string> visited;
  collectFileDescriptors(descriptor->file(), fds, visited);
  return fds.SerializeAsString();
}

// ─── Geodetic conversion ─────────────────────────────────────────────────────

struct GeoReference {
  double lat0, lon0, alt0;
};

void enuToLla(const gtsam::Point3& enu, const GeoReference& ref, double& lat, double& lon,
              double& alt) {
  constexpr double kDeg2Rad = M_PI / 180.0;
  constexpr double kR = 6'378'137.0;
  lat = ref.lat0 + enu.y() / (kR * kDeg2Rad);
  lon = ref.lon0 + enu.x() / (kR * std::cos(ref.lat0 * kDeg2Rad) * kDeg2Rad);
  alt = ref.alt0 + enu.z();
}

// ─── Timestamp helpers ───────────────────────────────────────────────────────

void setProtoTimestamp(google::protobuf::Timestamp* ts, uint64_t ns) {
  ts->set_seconds(static_cast<int64_t>(ns / 1'000'000'000ULL));
  ts->set_nanos(static_cast<int32_t>(ns % 1'000'000'000ULL));
}

// ─── Message builders ────────────────────────────────────────────────────────

std::string buildLocationFix(uint64_t stamp_ns, double lat, double lon, double alt) {
  foxglove::LocationFix msg;
  setProtoTimestamp(msg.mutable_timestamp(), stamp_ns);
  msg.set_frame_id("earth");
  msg.set_latitude(lat);
  msg.set_longitude(lon);
  msg.set_altitude(alt);
  return msg.SerializeAsString();
}

std::string buildFrameTransform(uint64_t stamp_ns, const std::string& parent,
                                const std::string& child, const gtsam::Pose3& pose) {
  foxglove::FrameTransform msg;
  setProtoTimestamp(msg.mutable_timestamp(), stamp_ns);
  msg.set_parent_frame_id(parent);
  msg.set_child_frame_id(child);

  const auto& t = pose.translation();
  msg.mutable_translation()->set_x(t.x());
  msg.mutable_translation()->set_y(t.y());
  msg.mutable_translation()->set_z(t.z());

  const auto q = pose.rotation().toQuaternion();
  msg.mutable_rotation()->set_w(q.w());
  msg.mutable_rotation()->set_x(q.x());
  msg.mutable_rotation()->set_y(q.y());
  msg.mutable_rotation()->set_z(q.z());

  return msg.SerializeAsString();
}

}  // namespace

// ─── Public API ──────────────────────────────────────────────────────────────

void writeResultsMcap(const std::filesystem::path& source_path,
                      const std::filesystem::path& output_path,
                      const results::OptimizedResults& results, const graph::FactorStorage& storage,
                      const domain::SceneData& scene) {
  // ─── Open source for reading ─────────────────────────────────────────────

  mcap::McapReader reader;
  {
    auto st = reader.open(source_path.string());
    if (!st.ok()) {
      std::cerr << "Failed to open source MCAP: " << st.message << "\n";
      return;
    }
  }

  // ─── Open output for writing ─────────────────────────────────────────────

  mcap::McapWriter writer;
  mcap::McapWriterOptions opts("nufuse");
  opts.compression = mcap::Compression::Zstd;
  opts.chunkSize = 32 * 1024 * 1024;  // 32 MB – avoids empty-chunk edge case for large messages

  {
    auto st = writer.open(output_path.string(), opts);
    if (!st.ok()) {
      std::cerr << "Failed to open output MCAP: " << st.message << "\n";
      reader.close();
      return;
    }
  }

  // ─── Copy source schemas and channels, build ID mapping ──────────────────

  {
    auto st = reader.readSummary(mcap::ReadSummaryMethod::NoFallbackScan);
    if (!st.ok()) {
      std::cerr << "Failed to read source summary: " << st.message << "\n";
      reader.close();
      writer.close();
      return;
    }
  }

  std::unordered_map<mcap::SchemaId, mcap::SchemaId> schema_map;
  mcap::SchemaId location_fix_schema_id = 0;
  mcap::SchemaId frame_transform_schema_id = 0;

  for (const auto& [src_id, src_schema_ptr] : reader.schemas()) {
    mcap::Schema schema(src_schema_ptr->name, src_schema_ptr->encoding, src_schema_ptr->data);
    writer.addSchema(schema);
    schema_map[src_id] = schema.id;

    // Reuse source schemas for nufuse channels (avoids duplicate FileDescriptorSets)
    if (src_schema_ptr->name == "foxglove.LocationFix" && src_schema_ptr->encoding == "protobuf") {
      location_fix_schema_id = schema.id;
    }
    if (src_schema_ptr->name == "foxglove.FrameTransform" &&
        src_schema_ptr->encoding == "protobuf") {
      frame_transform_schema_id = schema.id;
    }
  }

  // Fallback: register our own schemas if the source didn't have them
  if (location_fix_schema_id == 0) {
    const auto data = buildSchemaData(foxglove::LocationFix::descriptor());
    mcap::Schema schema("foxglove.LocationFix", "protobuf",
                        {reinterpret_cast<const std::byte*>(data.data()),
                         reinterpret_cast<const std::byte*>(data.data() + data.size())});
    writer.addSchema(schema);
    location_fix_schema_id = schema.id;
  }
  if (frame_transform_schema_id == 0) {
    const auto data = buildSchemaData(foxglove::FrameTransform::descriptor());
    mcap::Schema schema("foxglove.FrameTransform", "protobuf",
                        {reinterpret_cast<const std::byte*>(data.data()),
                         reinterpret_cast<const std::byte*>(data.data() + data.size())});
    writer.addSchema(schema);
    frame_transform_schema_id = schema.id;
  }

  std::unordered_map<mcap::ChannelId, mcap::ChannelId> channel_map;
  for (const auto& [src_id, src_channel_ptr] : reader.channels()) {
    mcap::Channel channel(src_channel_ptr->topic, src_channel_ptr->messageEncoding,
                          schema_map.at(src_channel_ptr->schemaId), src_channel_ptr->metadata);
    writer.addChannel(channel);
    channel_map[src_id] = channel.id;
  }

  // ─── Register nufuse channels (reusing source schemas) ───────────────────

  mcap::Channel gps_corrupted_ch("/nufuse/gps/corrupted", "protobuf", location_fix_schema_id);
  writer.addChannel(gps_corrupted_ch);

  mcap::Channel gps_clean_ch("/nufuse/gps/clean", "protobuf", location_fix_schema_id);
  writer.addChannel(gps_clean_ch);

  mcap::Channel pose_ch("/nufuse/pose", "protobuf", frame_transform_schema_id);
  writer.addChannel(pose_ch);

  mcap::Channel ext_estimated_ch("/nufuse/extrinsics/estimated", "protobuf",
                                 frame_transform_schema_id);
  writer.addChannel(ext_estimated_ch);

  mcap::Channel ext_gt_ch("/nufuse/extrinsics/ground_truth", "protobuf", frame_transform_schema_id);
  writer.addChannel(ext_gt_ch);

  mcap::Channel trajectory_ch("/nufuse/trajectory", "protobuf", location_fix_schema_id);
  writer.addChannel(trajectory_ch);

  mcap::Channel radar_ext_est_ch("/nufuse/radar_extrinsics/estimated", "protobuf",
                                 frame_transform_schema_id);
  writer.addChannel(radar_ext_est_ch);

  mcap::Channel radar_ext_gt_ch("/nufuse/radar_extrinsics/ground_truth", "protobuf",
                                frame_transform_schema_id);
  writer.addChannel(radar_ext_gt_ch);

  // ─── Pre-build nufuse messages sorted by timestamp ───────────────────────

  struct PendingMsg {
    uint64_t stamp_ns;
    mcap::ChannelId channel_id;
    std::string data;
  };
  std::vector<PendingMsg> nufuse_msgs;

  // GPS factors
  GeoReference ref{};
  if (!scene.gnss.empty()) {
    const auto& lla0 = scene.gnss.front().lla.value();
    ref.lat0 = lla0(0);
    ref.lon0 = lla0(1);
    ref.alt0 = lla0(2);
  }

  for (const auto& gps : storage.gps_factors) {
    double lat = 0, lon = 0, alt = 0;
    enuToLla(gps.position_enu.value(), ref, lat, lon, alt);
    nufuse_msgs.push_back({gps.stamp.value(), gps.corrupted ? gps_corrupted_ch.id : gps_clean_ch.id,
                           buildLocationFix(gps.stamp.value(), lat, lon, alt)});
  }

  // Poses + extrinsics
  const gtsam::Pose3& estimated_ext = results.body_from_lidar.value();
  const gtsam::Pose3 gt_ext = scene.extrinsics.body_from_lidar_top
                                  ? scene.extrinsics.body_from_lidar_top->value()
                                  : gtsam::Pose3::Identity();

  // Radar extrinsic ground truth lookup
  auto getRadarGtExtrinsic = [&](int idx) -> gtsam::Pose3 {
    switch (idx) {
      case 0:
        return scene.extrinsics.body_from_radar_front
                   ? scene.extrinsics.body_from_radar_front->value()
                   : gtsam::Pose3::Identity();
      case 1:
        return scene.extrinsics.body_from_radar_front_left
                   ? scene.extrinsics.body_from_radar_front_left->value()
                   : gtsam::Pose3::Identity();
      case 2:
        return scene.extrinsics.body_from_radar_front_right
                   ? scene.extrinsics.body_from_radar_front_right->value()
                   : gtsam::Pose3::Identity();
      case 3:
        return scene.extrinsics.body_from_radar_back_left
                   ? scene.extrinsics.body_from_radar_back_left->value()
                   : gtsam::Pose3::Identity();
      case 4:
        return scene.extrinsics.body_from_radar_back_right
                   ? scene.extrinsics.body_from_radar_back_right->value()
                   : gtsam::Pose3::Identity();
      default:
        return gtsam::Pose3::Identity();
    }
  };

  static const std::array<std::string, 5> kRadarNames = {"RADAR_FRONT", "RADAR_FRONT_LEFT",
                                                         "RADAR_FRONT_RIGHT", "RADAR_BACK_LEFT",
                                                         "RADAR_BACK_RIGHT"};

  for (const auto& kf : results.poses) {
    const uint64_t stamp_ns = kf.stamp.value();
    const gtsam::Pose3& body_pose = kf.pose.value();

    nufuse_msgs.push_back(
        {stamp_ns, pose_ch.id, buildFrameTransform(stamp_ns, "map", "base_link", body_pose)});

    // Lidar extrinsics (only if lidar was active)
    if (scene.extrinsics.body_from_lidar_top) {
      nufuse_msgs.push_back(
          {stamp_ns, ext_estimated_ch.id,
           buildFrameTransform(stamp_ns, "base_link", "LIDAR_TOP_estimated", estimated_ext)});
      nufuse_msgs.push_back({stamp_ns, ext_gt_ch.id,
                             buildFrameTransform(stamp_ns, "base_link", "LIDAR_TOP_gt", gt_ext)});
    }

    // Radar extrinsics
    for (const auto& [id, est_pose] : results.radar_extrinsics) {
      const int idx = static_cast<int>(id);
      if (idx >= 0 && idx < 5) {
        const std::string est_child = kRadarNames[idx] + "_estimated";
        const std::string gt_child = kRadarNames[idx] + "_gt";
        nufuse_msgs.push_back({stamp_ns, radar_ext_est_ch.id,
                               buildFrameTransform(stamp_ns, "base_link", est_child, est_pose)});
        nufuse_msgs.push_back(
            {stamp_ns, radar_ext_gt_ch.id,
             buildFrameTransform(stamp_ns, "base_link", gt_child, getRadarGtExtrinsic(idx))});
      }
    }

    // Estimated trajectory as LocationFix for map panel
    double traj_lat = 0, traj_lon = 0, traj_alt = 0;
    enuToLla(body_pose.translation(), ref, traj_lat, traj_lon, traj_alt);
    nufuse_msgs.push_back(
        {stamp_ns, trajectory_ch.id, buildLocationFix(stamp_ns, traj_lat, traj_lon, traj_alt)});
  }

  std::sort(nufuse_msgs.begin(), nufuse_msgs.end(),
            [](const PendingMsg& a, const PendingMsg& b) { return a.stamp_ns < b.stamp_ns; });

  // ─── Merge-write source + nufuse messages in timestamp order ─────────────

  auto view = reader.readMessages();
  size_t nufuse_idx = 0;
  uint64_t copied = 0;

  auto flushNufuseUpto = [&](uint64_t up_to_ns) {
    while (nufuse_idx < nufuse_msgs.size() && nufuse_msgs[nufuse_idx].stamp_ns <= up_to_ns) {
      const auto& pm = nufuse_msgs[nufuse_idx];
      mcap::Message msg;
      msg.channelId = pm.channel_id;
      msg.sequence = 0;
      msg.logTime = pm.stamp_ns;
      msg.publishTime = pm.stamp_ns;
      msg.data = reinterpret_cast<const std::byte*>(pm.data.data());
      msg.dataSize = pm.data.size();
      (void)writer.write(msg);
      ++nufuse_idx;
    }
  };

  for (auto it = view.begin(); it != view.end(); ++it) {
    const auto& src_msg = it->message;

    // Flush nufuse messages that belong before this source message
    flushNufuseUpto(src_msg.logTime);

    mcap::Message msg;
    msg.channelId = channel_map.at(src_msg.channelId);
    msg.sequence = src_msg.sequence;
    msg.logTime = src_msg.logTime;
    msg.publishTime = src_msg.publishTime;
    msg.data = src_msg.data;
    msg.dataSize = src_msg.dataSize;
    (void)writer.write(msg);
    ++copied;
  }

  // Flush any remaining nufuse messages after the last source message
  flushNufuseUpto(UINT64_MAX);

  // ─── Done ────────────────────────────────────────────────────────────────

  reader.close();
  writer.close();
  std::cout << "\nMerged MCAP written to: " << output_path << "\n"
            << "  Source messages copied: " << copied << "\n"
            << "  GPS factors: " << storage.gps_factors.size() << " (" << storage.num_corrupted
            << " corrupted)\n"
            << "  Keyframes: " << results.poses.size() << "\n";
}

}  // namespace nufuse::io
