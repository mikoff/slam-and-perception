#include "dataset_parser.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

void parseDatasetSE2(const std::string& path,
                     std::vector<RawEdge2>& edges,
                     std::vector<RawLandmark2>& landmarks) {
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open dataset: " + path);
  }

  std::string line;
  while (std::getline(file, line)) {
    std::istringstream iss(line);
    std::string type;
    iss >> type;

    // Accept both EDGE2 and ODOMETRY as SE2 odometry edges.
    if (type == "EDGE2" || type == "ODOMETRY") {
      RawEdge2 e{};
      double i11, i12, i13, i22, i23, i33;
      iss >> e.id_from >> e.id_to >> e.dx >> e.dy >> e.dtheta
          >> i11 >> i12 >> i13 >> i22 >> i23 >> i33;
      e.info << i11, i12, i13, i12, i22, i23, i13, i23, i33;
      edges.push_back(e);
    } else if (type == "LANDMARK") {
      RawLandmark2 lm{};
      double i11, i12, i22;
      iss >> lm.pose_id >> lm.lm_id >> lm.dx >> lm.dy >> i11 >> i12 >> i22;
      lm.info << i11, i12, i12, i22;
      landmarks.push_back(lm);
    }
  }
}

// ---------------------------------------------------------------------------
// SE3 parser — handles EDGE3 and POINT3 tokens.
//
// EDGE3 format:
//   EDGE3 id_from id_to  tx ty tz  rx ry rz
//         i11 i12 i13 i14 i15 i16
//             i22 i23 i24 i25 i26
//                 i33 i34 i35 i36
//                     i44 i45 i46
//                         i55 i56
//                             i66
//   where (tx,ty,tz) = translation, (rx,ry,rz) = axis-angle rotation.
//   Info matrix ordering: (tx, ty, tz, rx, ry, rz).
//
// POINT3 format:
//   POINT3 pose_id lm_id  dx dy dz  i11 i12 i13 i22 i23 i33
// ---------------------------------------------------------------------------

void parseDatasetSE3(const std::string& path,
                     std::vector<RawEdge3>& edges,
                     std::vector<RawLandmark3>& landmarks) {
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open dataset: " + path);
  }

  std::string line;
  while (std::getline(file, line)) {
    std::istringstream iss(line);
    std::string type;
    iss >> type;

    if (type == "EDGE3") {
      RawEdge3 e{};
      iss >> e.id_from >> e.id_to
          >> e.tx >> e.ty >> e.tz
          >> e.rx >> e.ry >> e.rz;

      // Read 21 upper-triangular elements of the 6×6 info matrix.
      double v[21];
      for (int i = 0; i < 21; ++i) iss >> v[i];

      e.info.setZero();
      int idx = 0;
      for (int r = 0; r < 6; ++r)
        for (int c = r; c < 6; ++c) {
          e.info(r, c) = v[idx];
          e.info(c, r) = v[idx];
          ++idx;
        }
      edges.push_back(e);

    } else if (type == "POINT3") {
      RawLandmark3 lm{};
      double i11, i12, i13, i22, i23, i33;
      iss >> lm.pose_id >> lm.lm_id
          >> lm.dx >> lm.dy >> lm.dz
          >> i11 >> i12 >> i13 >> i22 >> i23 >> i33;
      lm.info << i11, i12, i13,
                 i12, i22, i23,
                 i13, i23, i33;
      landmarks.push_back(lm);
    }
  }
}

