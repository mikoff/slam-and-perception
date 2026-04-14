#pragma once

// ============================================================================
// Dataset parser for iSAM2-format files (SE2 and SE3 geometry).
//
// SE2: reads EDGE2 / ODOMETRY / LANDMARK lines.
// SE3: reads EDGE3 / POINT3 lines.
// ============================================================================

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include <sym/pose2.h>
#include <sym/pose3.h>
#include <sym/rot2.h>
#include <sym/rot3.h>

// ---- Raw measurement types (SE2 dataset format) -------------------------

struct RawEdge2 {
  int id_from, id_to;
  double dx, dy, dtheta;
  Eigen::Matrix3d info;
};

struct RawLandmark2 {
  int pose_id, lm_id;
  double dx, dy;
  Eigen::Matrix2d info;
};

// ---- Raw measurement types (SE3 dataset format) -------------------------

// Translation (tx,ty,tz) + axis-angle rotation (rx,ry,rz) + 6×6 upper-tri info.
// Info is stored in the g2o/iSAM2 ordering: (tx,ty,tz,rx,ry,rz).
struct RawEdge3 {
  int id_from, id_to;
  double tx, ty, tz;    // translation
  double rx, ry, rz;    // rotation as axis-angle
  Eigen::Matrix<double, 6, 6> info;  // upper-triangular, g2o ordering
};

struct RawLandmark3 {
  int pose_id, lm_id;
  double dx, dy, dz;    // observation in body frame
  Eigen::Matrix3d info;
};

// ---- Parsing ------------------------------------------------------------

// Both EDGE2 and ODOMETRY tokens are treated as odometry edges.
void parseDatasetSE2(const std::string& path,
                     std::vector<RawEdge2>& edges,
                     std::vector<RawLandmark2>& landmarks);

void parseDatasetSE3(const std::string& path,
                     std::vector<RawEdge3>& edges,
                     std::vector<RawLandmark3>& landmarks);

// ---- Dead-reckoning (generic) -------------------------------------------
// Chains sequential odometry edges (id_to == id_from + 1) to produce
// initial pose estimates.

inline std::vector<sym::Pose2d> deadReckonSE2(
    const std::vector<RawEdge2>& edges, int num_poses) {
  using Pose = sym::Pose2d;
  std::vector<Pose> poses(num_poses, Pose::Identity());

  std::vector<const RawEdge2*> odom;
  for (const auto& e : edges)
    if (e.id_to == e.id_from + 1) odom.push_back(&e);

  std::sort(odom.begin(), odom.end(),
            [](auto* a, auto* b) { return a->id_from < b->id_from; });

  for (const auto* e : odom) {
    if (e->id_from < num_poses && e->id_to < num_poses) {
      poses[e->id_to] = poses[e->id_from] *
          sym::Pose2d(sym::Rot2d(e->dtheta), Eigen::Vector2d(e->dx, e->dy));
    }
  }
  return poses;
}

// ---- Landmark initialization (SE2) --------------------------------------
// Projects each landmark's first observation into the world frame.

inline std::unordered_map<int, Eigen::Vector2d> initLandmarksSE2(
    const std::vector<RawLandmark2>& obs,
    const std::vector<sym::Pose2d>& poses) {
  std::unordered_map<int, Eigen::Vector2d> lm;
  for (const auto& o : obs)
    if (lm.count(o.lm_id) == 0)
      lm[o.lm_id] = poses[o.pose_id] * Eigen::Vector2d(o.dx, o.dy);
  return lm;
}

// ---- Dead-reckoning (SE3) -----------------------------------------------

inline sym::Rot3d rot3FromAxisAngle(double rx, double ry, double rz) {
  double angle = std::sqrt(rx * rx + ry * ry + rz * rz);
  if (angle < 1e-10) return sym::Rot3d::Identity();
  Eigen::Quaterniond q(
      Eigen::AngleAxisd(angle, Eigen::Vector3d(rx / angle, ry / angle, rz / angle)));
  return sym::Rot3d(q);
}

inline std::vector<sym::Pose3d> deadReckonSE3(
    const std::vector<RawEdge3>& edges, int num_poses) {
  std::vector<sym::Pose3d> poses(num_poses, sym::Pose3d::Identity());

  std::vector<const RawEdge3*> odom;
  for (const auto& e : edges)
    if (e.id_to == e.id_from + 1) odom.push_back(&e);

  std::sort(odom.begin(), odom.end(),
            [](auto* a, auto* b) { return a->id_from < b->id_from; });

  for (const auto* e : odom) {
    if (e->id_from < num_poses && e->id_to < num_poses) {
      sym::Pose3d rel(rot3FromAxisAngle(e->rx, e->ry, e->rz),
                      Eigen::Vector3d(e->tx, e->ty, e->tz));
      poses[e->id_to] = poses[e->id_from] * rel;
    }
  }
  return poses;
}

// ---- Landmark initialization (SE3) --------------------------------------

inline std::unordered_map<int, Eigen::Vector3d> initLandmarksSE3(
    const std::vector<RawLandmark3>& obs,
    const std::vector<sym::Pose3d>& poses) {
  std::unordered_map<int, Eigen::Vector3d> lm;
  for (const auto& o : obs)
    if (lm.count(o.lm_id) == 0)
      lm[o.lm_id] = poses[o.pose_id] * Eigen::Vector3d(o.dx, o.dy, o.dz);
  return lm;
}

// ---- Geometry auto-detection --------------------------------------------
// Peeks at the first recognized token in the file.

inline std::string detectDatasetGeometry(const std::string& path) {
  std::ifstream f(path);
  std::string line, token;
  while (std::getline(f, line)) {
    std::istringstream iss(line);
    if (iss >> token) {
      if (token == "EDGE3" || token == "POINT3") return "SE3";
      if (token == "EDGE2" || token == "LANDMARK" || token == "ODOMETRY") return "SE2";
    }
  }
  return "SE2";
}
