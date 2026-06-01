/// @file core/config_loader.hpp
/// @brief Load PipelineConfig from a JSON file.

#pragma once

#include <filesystem>

#include "core/pipeline_config.hpp"

namespace nufuse::core {

/// @brief Load configuration from a JSON file.
/// Fields not present in the JSON retain their compiled defaults.
PipelineConfig loadConfig(const std::filesystem::path& json_path);

}  // namespace nufuse::core
