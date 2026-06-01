/// @file io/mcap_writer.hpp
/// @brief Writes optimization results to MCAP for Foxglove/Lichtblick visualization.

#pragma once

#include <filesystem>

#include "domain/scene_data.hpp"
#include "graph/factor_storage.hpp"
#include "results/optimizer.hpp"

namespace nufuse::io {

/// @brief Merges original MCAP data with optimization results into a single output file.
/// @param source_path Path to the original input MCAP (all messages are copied).
/// @param output_path Path for the merged output .mcap file.
/// @param results Optimized trajectory and extrinsics.
/// @param storage Factor storage with GPS corruption flags.
/// @param scene Original scene data (for reference LLA and ground truth extrinsics).
void writeResultsMcap(const std::filesystem::path& source_path,
                      const std::filesystem::path& output_path,
                      const results::OptimizedResults& results, const graph::FactorStorage& storage,
                      const domain::SceneData& scene);

}  // namespace nufuse::io
