/// @file bindings/pynufuse.cpp
/// @brief pybind11 Python bindings for the NuFuse SLAM pipeline.

#include <filesystem>
#include <iostream>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <sstream>
#include <stdexcept>

#include "core/config_loader.hpp"
#include "core/pipeline_config.hpp"
#include "domain/scene_data.hpp"
#include "io/mcap_loader.hpp"
#include "pipeline/pipeline.hpp"
#include "results/optimizer.hpp"

namespace py = pybind11;

namespace {

/// RAII helper to suppress stdout/stderr during pipeline execution.
struct SuppressOutput {
  std::streambuf* cout_buf;
  std::streambuf* cerr_buf;
  std::ostringstream sink;

  SuppressOutput() : cout_buf(std::cout.rdbuf()), cerr_buf(std::cerr.rdbuf()) {
    std::cout.rdbuf(sink.rdbuf());
    std::cerr.rdbuf(sink.rdbuf());
  }
  ~SuppressOutput() {
    std::cout.rdbuf(cout_buf);
    std::cerr.rdbuf(cerr_buf);
  }
};

/// Compute end-position error given results and scene.
struct ErrorMetrics {
  Eigen::Vector3d ground_truth;
  Eigen::Vector3d estimated;
  double norm_error;
};

ErrorMetrics computeError(const nufuse::results::OptimizedResults& res,
                          const nufuse::domain::SceneData& scene, bool init_from_gt) {
  const gtsam::Point3 gt_disp =
      scene.odom.back().pose.value().translation() - scene.odom.front().pose.value().translation();
  const gtsam::Point3 odom_start = scene.odom.front().pose.value().translation();
  const gtsam::Point3 estimated = res.poses.back().pose.value().translation();
  const gtsam::Point3 est_disp = init_from_gt ? (estimated - odom_start) : estimated;
  const gtsam::Point3 error = est_disp - gt_disp;

  return {gt_disp, est_disp, error.norm()};
}

}  // anonymous namespace

PYBIND11_MODULE(_nufuse, m) {
  m.doc() = "Python bindings for the NuFuse multi-sensor SLAM pipeline";

  // =========================================================================
  // PipelineConfig
  // =========================================================================
  py::enum_<nufuse::core::RobustKernelType>(m, "RobustKernel")
      .value("Huber", nufuse::core::RobustKernelType::Huber)
      .value("Cauchy", nufuse::core::RobustKernelType::Cauchy);

  py::enum_<nufuse::core::OptimizerType>(m, "OptimizerType")
      .value("LM", nufuse::core::OptimizerType::LM)
      .value("GNC_LM", nufuse::core::OptimizerType::GNC_LM);

  py::class_<nufuse::core::PipelineConfig::GncConfig>(m, "GncConfig")
      .def(py::init<>())
      .def_readwrite("mu_step", &nufuse::core::PipelineConfig::GncConfig::mu_step)
      .def_readwrite("relative_cost_tol",
                     &nufuse::core::PipelineConfig::GncConfig::relative_cost_tol)
      .def_readwrite("weights_tol", &nufuse::core::PipelineConfig::GncConfig::weights_tol);

  py::class_<nufuse::core::LidarNoiseConfig>(m, "LidarConfig")
      .def(py::init<>())
      .def_readwrite("odom_rot_sigma", &nufuse::core::LidarNoiseConfig::odom_rot_sigma)
      .def_readwrite("odom_trans_sigma", &nufuse::core::LidarNoiseConfig::odom_trans_sigma)
      .def_readwrite("robust_threshold", &nufuse::core::LidarNoiseConfig::robust_threshold)
      .def_readwrite("min_inliers", &nufuse::core::LidarNoiseConfig::min_inliers)
      .def_readwrite("max_registration_error",
                     &nufuse::core::LidarNoiseConfig::max_registration_error)
      .def_readwrite("max_translation_m", &nufuse::core::LidarNoiseConfig::max_translation_m)
      .def_readwrite("imu_consistency_threshold",
                     &nufuse::core::LidarNoiseConfig::imu_consistency_threshold);

  py::class_<nufuse::core::RadarNoiseConfig>(m, "RadarConfig")
      .def(py::init<>())
      .def_readwrite("velocity_sigma", &nufuse::core::RadarNoiseConfig::velocity_sigma)
      .def_readwrite("robust_threshold", &nufuse::core::RadarNoiseConfig::robust_threshold);

  py::class_<nufuse::core::PipelineConfig>(m, "PipelineConfig")
      .def(py::init<>())
      .def_readwrite("enable_gps", &nufuse::core::PipelineConfig::enable_gps)
      .def_readwrite("enable_lidar", &nufuse::core::PipelineConfig::enable_lidar)
      .def_readwrite("enable_radar", &nufuse::core::PipelineConfig::enable_radar)
      .def_readwrite("enable_nhc", &nufuse::core::PipelineConfig::enable_nhc)
      .def_readwrite("enable_fwdvel", &nufuse::core::PipelineConfig::enable_fwdvel)
      .def_readwrite("gps_max_measurements", &nufuse::core::PipelineConfig::gps_max_measurements)
      .def_readwrite("robust_kernel", &nufuse::core::PipelineConfig::robust_kernel)
      .def_readwrite("optimizer", &nufuse::core::PipelineConfig::optimizer)
      .def_readwrite("gnc", &nufuse::core::PipelineConfig::gnc)
      .def_readwrite("lidar", &nufuse::core::PipelineConfig::lidar)
      .def_readwrite("radar", &nufuse::core::PipelineConfig::radar)
      .def("__repr__", [](const nufuse::core::PipelineConfig& c) {
        std::ostringstream os;
        os << "PipelineConfig(gps=" << c.enable_gps << ", lidar=" << c.enable_lidar
           << ", radar=" << c.enable_radar << ", nhc=" << c.enable_nhc
           << ", fwdvel=" << c.enable_fwdvel << ", gps_max=" << c.gps_max_measurements
           << ", optimizer="
           << (c.optimizer == nufuse::core::OptimizerType::GNC_LM ? "GNC_LM" : "LM") << ", kernel="
           << (c.robust_kernel == nufuse::core::RobustKernelType::Cauchy ? "Cauchy" : "Huber")
           << ")";
        return os.str();
      });

  // =========================================================================
  // OptimizedPose (lightweight view)
  // =========================================================================
  py::class_<nufuse::results::OptimizedPose>(m, "OptimizedPose")
      .def_property_readonly(
          "stamp_ns", [](const nufuse::results::OptimizedPose& p) { return p.stamp.value(); })
      .def_property_readonly("translation",
                             [](const nufuse::results::OptimizedPose& p) {
                               return Eigen::Vector3d(p.pose.value().translation());
                             })
      .def_property_readonly("rotation_matrix",
                             [](const nufuse::results::OptimizedPose& p) {
                               return p.pose.value().rotation().matrix();
                             })
      .def_property_readonly("covariance",
                             [](const nufuse::results::OptimizedPose& p) { return p.covariance; });

  // =========================================================================
  // OptimizedResults
  // =========================================================================
  py::class_<nufuse::results::OptimizedResults>(m, "OptimizedResults")
      .def_readonly("poses", &nufuse::results::OptimizedResults::poses)
      .def_readonly("velocities", &nufuse::results::OptimizedResults::velocities)
      .def_readonly("num_keyframes", &nufuse::results::OptimizedResults::num_keyframes)
      .def_readonly("num_corrupted_gnss", &nufuse::results::OptimizedResults::num_corrupted_gnss)
      .def_property_readonly("lidar_extrinsics_translation",
                             [](const nufuse::results::OptimizedResults& r) {
                               return Eigen::Vector3d(r.body_from_lidar.value().translation());
                             })
      .def_property_readonly(
          "lidar_extrinsics_covariance",
          [](const nufuse::results::OptimizedResults& r) { return r.lidar_extrinsics_covariance; })
      .def_property_readonly(
          "odom_scale",
          [](const nufuse::results::OptimizedResults& r) { return 1.0 + r.odom_scale_delta.value; })
      .def_property_readonly("final_bias_accel",
                             [](const nufuse::results::OptimizedResults& r) {
                               return Eigen::Vector3d(r.biases.back().bias.accelerometer());
                             })
      .def_property_readonly("final_bias_gyro", [](const nufuse::results::OptimizedResults& r) {
        return Eigen::Vector3d(r.biases.back().bias.gyroscope());
      });

  // =========================================================================
  // ErrorMetrics
  // =========================================================================
  py::class_<ErrorMetrics>(m, "ErrorMetrics")
      .def_readonly("ground_truth", &ErrorMetrics::ground_truth)
      .def_readonly("estimated", &ErrorMetrics::estimated)
      .def_readonly("norm_error", &ErrorMetrics::norm_error);

  // =========================================================================
  // SceneInfo — lightweight metadata about a loaded scene (no heavy data)
  // =========================================================================
  struct SceneInfo {
    size_t num_imu, num_gnss, num_odom, num_lidar, num_radar;
    bool has_lidar_extrinsics;
  };

  py::class_<SceneInfo>(m, "SceneInfo")
      .def_readonly("num_imu", &SceneInfo::num_imu)
      .def_readonly("num_gnss", &SceneInfo::num_gnss)
      .def_readonly("num_odom", &SceneInfo::num_odom)
      .def_readonly("num_lidar", &SceneInfo::num_lidar)
      .def_readonly("num_radar", &SceneInfo::num_radar)
      .def_readonly("has_lidar_extrinsics", &SceneInfo::has_lidar_extrinsics);

  // =========================================================================
  // PipelineResult — bundles results + error metrics for convenience
  // =========================================================================
  struct PipelineResult {
    nufuse::results::OptimizedResults results;
    ErrorMetrics error;
    SceneInfo scene_info;
  };

  py::class_<PipelineResult>(m, "PipelineResult")
      .def_readonly("results", &PipelineResult::results)
      .def_readonly("error", &PipelineResult::error)
      .def_readonly("scene_info", &PipelineResult::scene_info);

  // =========================================================================
  // Top-level functions
  // =========================================================================

  m.def(
      "load_config",
      [](const std::string& json_path) {
        return nufuse::core::loadConfig(std::filesystem::path(json_path));
      },
      py::arg("json_path"), "Load a PipelineConfig from a JSON file.");

  m.def(
      "run_pipeline",
      [](const std::string& mcap_path, const nufuse::core::PipelineConfig& config,
         bool init_from_gt, bool verbose) -> PipelineResult {
        const std::filesystem::path path(mcap_path);
        if (!std::filesystem::exists(path)) {
          throw std::runtime_error("File not found: " + mcap_path);
        }

        // Load scene
        auto scene = nufuse::io::loadMcap(path);
        if (scene.imu.empty() || scene.gnss.size() < 2 || scene.odom.empty()) {
          throw std::runtime_error("Insufficient data in MCAP file");
        }

        SceneInfo info{scene.imu.size(),   scene.gnss.size(),
                       scene.odom.size(),  scene.lidar.size(),
                       scene.radar.size(), scene.extrinsics.body_from_lidar_top.has_value()};

        // Run pipeline (optionally suppress output)
        nufuse::results::OptimizedResults results;
        if (verbose) {
          results = nufuse::pipeline::run(scene, config, init_from_gt);
        } else {
          SuppressOutput guard;
          results = nufuse::pipeline::run(scene, config, init_from_gt);
        }
        auto error = computeError(results, scene, init_from_gt);

        return PipelineResult{std::move(results), error, info};
      },
      py::arg("mcap_path"), py::arg("config") = nufuse::core::PipelineConfig{},
      py::arg("init_from_gt") = false, py::arg("verbose") = false,
      "Run the full NuFuse pipeline on an MCAP file and return results + error metrics.");

  m.def(
      "run_batch",
      [](const std::vector<std::string>& mcap_paths, const nufuse::core::PipelineConfig& config,
         bool init_from_gt, bool verbose) -> std::vector<std::pair<std::string, PipelineResult>> {
        std::vector<std::pair<std::string, PipelineResult>> batch_results;
        batch_results.reserve(mcap_paths.size());

        for (const auto& mcap_path : mcap_paths) {
          const std::filesystem::path path(mcap_path);
          if (!std::filesystem::exists(path)) {
            throw std::runtime_error("File not found: " + mcap_path);
          }

          auto scene = nufuse::io::loadMcap(path);
          if (scene.imu.empty() || scene.gnss.size() < 2 || scene.odom.empty()) {
            throw std::runtime_error("Insufficient data in: " + mcap_path);
          }

          SceneInfo info{scene.imu.size(),   scene.gnss.size(),
                         scene.odom.size(),  scene.lidar.size(),
                         scene.radar.size(), scene.extrinsics.body_from_lidar_top.has_value()};

          nufuse::results::OptimizedResults results;
          if (verbose) {
            results = nufuse::pipeline::run(scene, config, init_from_gt);
          } else {
            SuppressOutput guard;
            results = nufuse::pipeline::run(scene, config, init_from_gt);
          }
          auto error = computeError(results, scene, init_from_gt);

          // Extract scene ID from filename
          std::string scene_id = path.stem().string();
          auto pos = scene_id.rfind('-');
          if (pos != std::string::npos) scene_id = scene_id.substr(pos + 1);

          batch_results.emplace_back(scene_id, PipelineResult{std::move(results), error, info});
        }
        return batch_results;
      },
      py::arg("mcap_paths"), py::arg("config") = nufuse::core::PipelineConfig{},
      py::arg("init_from_gt") = false, py::arg("verbose") = false,
      "Run the pipeline on multiple MCAP files and return a list of (scene_id, result) pairs.");
}
