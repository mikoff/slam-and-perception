/// @file pose_graph_optimizer.cpp
/// @brief Pose-graph optimizer — SE2 and SE3 SLAM with landmarks.
///
/// Geometry is auto-detected from the dataset file:
///   EDGE2 / ODOMETRY / LANDMARK  →  SE2 pipeline
///   EDGE3 / POINT3               →  SE3 pipeline

#include <algorithm>
#include <string>
#include <unordered_map>
#include <vector>

#include <spdlog/spdlog.h>

#include "dataset_parser.hpp"
#include "factor_graph.hpp"

namespace {

std::string makeOutputPath(const std::string& dataset_path,
                           const std::string& output_dir) {
  auto slash = dataset_path.find_last_of("/\\");
  std::string basename =
      (slash == std::string::npos) ? dataset_path : dataset_path.substr(slash + 1);
  auto dot = basename.rfind('.');
  if (dot != std::string::npos) basename = basename.substr(0, dot);
  std::string dir = output_dir;
  if (!dir.empty() && dir.back() != '/') dir += '/';
  return dir + basename + ".json";
}

// ---------------------------------------------------------------------------
// SE2 pipeline
// ---------------------------------------------------------------------------
int runSE2(const std::string& dataset_path, const std::string& output_path) {
  using G = SE2;

  std::vector<RawEdge2> edges;
  std::vector<RawLandmark2> landmarks;
  parseDatasetSE2(dataset_path, edges, landmarks);

  int num_poses = 0;
  for (const auto& e : edges)
    num_poses = std::max(num_poses, std::max(e.id_from, e.id_to) + 1);
  for (const auto& lm : landmarks)
    num_poses = std::max(num_poses, lm.pose_id + 1);

  spdlog::info("[SE2] Parsed {} edges, {} landmark obs, {} poses",
               edges.size(), landmarks.size(), num_poses);

  auto init_poses = deadReckonSE2(edges, num_poses);
  auto init_lm    = initLandmarksSE2(landmarks, init_poses);
  spdlog::info("[SE2] Initialized {} landmarks", init_lm.size());

  CFactorGraph<G> graph;
  auto& s = graph.storage();

  for (int i = 0; i < num_poses; ++i)
    s.setPose(PoseId(i), init_poses[i]);
  for (const auto& [lm_id, pos] : init_lm)
    s.setLandmark(LandmarkId(lm_id), pos);

  s.addPrior({.pose_id    = PoseId(0),
              .prior_pose = G::Pose::Identity(),
              .info       = G::PoseInfoMatrix::Identity() * 1000.0});

  for (const auto& e : edges) {
    s.addBetween({.id_from       = PoseId(e.id_from),
                  .id_to         = PoseId(e.id_to),
                  .relative_pose = G::Pose(sym::Rot2d(e.dtheta),
                                           Eigen::Vector2d(e.dx, e.dy)),
                  .info          = e.info});
  }

  for (const auto& o : landmarks) {
    s.addLandmarkObservation({.pose_id      = PoseId(o.pose_id),
                              .landmark_id  = LandmarkId(o.lm_id),
                              .observation  = Eigen::Vector2d(o.dx, o.dy),
                              .info         = o.info,
                              .barron_alpha = 1.0});
  }

  spdlog::info("[SE2] Graph: {} factors ({} edges, {} lm obs, {} priors)",
               s.numFactors(), s.numEdges(),
               s.numLandmarkObservations(), s.numPriors());

  auto result = graph.optimize();
  graph.saveJson(output_path, result);
  return 0;
}

// ---------------------------------------------------------------------------
// SE3 pipeline
// ---------------------------------------------------------------------------
int runSE3(const std::string& dataset_path, const std::string& output_path) {
  using G = SE3;

  std::vector<RawEdge3> edges;
  std::vector<RawLandmark3> landmarks;
  parseDatasetSE3(dataset_path, edges, landmarks);

  int num_poses = 0;
  for (const auto& e : edges)
    num_poses = std::max(num_poses, std::max(e.id_from, e.id_to) + 1);
  for (const auto& lm : landmarks)
    num_poses = std::max(num_poses, lm.pose_id + 1);

  spdlog::info("[SE3] Parsed {} edges, {} landmark obs, {} poses",
               edges.size(), landmarks.size(), num_poses);

  auto init_poses = deadReckonSE3(edges, num_poses);
  auto init_lm    = initLandmarksSE3(landmarks, init_poses);
  spdlog::info("[SE3] Initialized {} landmarks", init_lm.size());

  CFactorGraph<G> graph;
  auto& s = graph.storage();

  for (int i = 0; i < num_poses; ++i)
    s.setPose(PoseId(i), init_poses[i]);
  for (const auto& [lm_id, pos] : init_lm)
    s.setLandmark(LandmarkId(lm_id), pos);

  s.addPrior({.pose_id    = PoseId(0),
              .prior_pose = G::Pose::Identity(),
              .info       = G::PoseInfoMatrix::Identity() * 1000.0});

  for (const auto& e : edges) {
    // g2o/iSAM2 EDGE3 info ordering: (tx,ty,tz,rx,ry,rz).
    // Symforce SE3 tangent ordering:  (rx,ry,rz,tx,ty,tz).
    // Permute the 6×6 info matrix accordingly.
    static const int perm[6] = {3, 4, 5, 0, 1, 2};
    G::PoseInfoMatrix info_sf = G::PoseInfoMatrix::Zero();
    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 6; ++j)
        info_sf(i, j) = e.info(perm[i], perm[j]);

    s.addBetween({.id_from       = PoseId(e.id_from),
                  .id_to         = PoseId(e.id_to),
                  .relative_pose = G::Pose(rot3FromAxisAngle(e.rx, e.ry, e.rz),
                                           Eigen::Vector3d(e.tx, e.ty, e.tz)),
                  .info          = info_sf});
  }

  for (const auto& o : landmarks) {
    s.addLandmarkObservation({.pose_id      = PoseId(o.pose_id),
                              .landmark_id  = LandmarkId(o.lm_id),
                              .observation  = Eigen::Vector3d(o.dx, o.dy, o.dz),
                              .info         = o.info,
                              .barron_alpha = 1.0});
  }

  spdlog::info("[SE3] Graph: {} factors ({} edges, {} lm obs, {} priors)",
               s.numFactors(), s.numEdges(),
               s.numLandmarkObservations(), s.numPriors());

  auto result = graph.optimize();
  graph.saveJson(output_path, result);
  return 0;
}

}  // namespace

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
  if (argc < 2) {
    spdlog::error("Usage: {} <dataset_path> [output_dir]", argv[0]);
    return 1;
  }
  const std::string dataset_path = argv[1];
  const std::string output_dir   = (argc > 2) ? argv[2] : ".";
  const std::string output_path  = makeOutputPath(dataset_path, output_dir);

  const std::string geom = detectDatasetGeometry(dataset_path);
  spdlog::info("Dataset: {}  →  geometry: {}", dataset_path, geom);
  spdlog::info("Output:  {}", output_path);

  if (geom == "SE3") return runSE3(dataset_path, output_path);
  return runSE2(dataset_path, output_path);
}

