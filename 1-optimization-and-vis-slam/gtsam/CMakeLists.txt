cmake_minimum_required(VERSION 3.14)
project(GtsamProject)
set(CMAKE_CXX_STANDARD 20)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(GTSAM REQUIRED)

# Consume slam-types header-only library from sibling directory
add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../slam-types
    ${CMAKE_CURRENT_BINARY_DIR}/slam-types
    EXCLUDE_FROM_ALL
)

add_executable(gtsam_example src/main.cpp)
add_executable(safe_types_example src/test_gtsam_geometry.cpp)

target_link_libraries(gtsam_example PRIVATE gtsam)
target_link_libraries(safe_types_example PRIVATE gtsam slam_types)

