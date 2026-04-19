cmake_minimum_required(VERSION 3.14)
project(Week1Symforce)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_definitions(FMT_CONSTEVAL=)

# Release build with -O3 by default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# Fetch SymForce - only build the C++ optimization library
set(SYMFORCE_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(SYMFORCE_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(SYMFORCE_BUILD_CC_SYM OFF CACHE BOOL "" FORCE)
set(SYMFORCE_BUILD_SYMENGINE OFF CACHE BOOL "" FORCE)

include(FetchContent)
FetchContent_Declare(
  symforce
  GIT_REPOSITORY https://github.com/symforce-org/symforce.git
  GIT_TAG v0.10.1
)
FetchContent_MakeAvailable(symforce)

FetchContent_Declare(json
  URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz)
FetchContent_MakeAvailable(json)

# ---- Pose-graph optimizer ----
add_executable(pose_graph_optimizer
  src/pose_graph_optimizer.cpp
  src/dataset_parser.cpp
  src/geometry_types.cpp
  src/factor_graph_storage.cpp
  src/factor_graph.cpp
)
target_link_libraries(pose_graph_optimizer PRIVATE symforce_opt nlohmann_json::nlohmann_json)
target_include_directories(pose_graph_optimizer PRIVATE
  ${CMAKE_SOURCE_DIR}/inc/gen
  ${CMAKE_SOURCE_DIR}/inc
  ${CMAKE_SOURCE_DIR}/../slam-types/include
)