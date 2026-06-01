/// @file io/mcap_loader.hpp
/// @brief Reads NuScenes MCAP files and returns typed sensor data.

#pragma once

#include "domain/scene_data.hpp"

#include <filesystem>

namespace nufuse::io {

/// @brief Loads all supported sensor data from a NuScenes MCAP file.
/// @throws std::runtime_error if the file cannot be opened.
domain::SceneData loadMcap(const std::filesystem::path& mcap_path);

}  // namespace nufuse::io
