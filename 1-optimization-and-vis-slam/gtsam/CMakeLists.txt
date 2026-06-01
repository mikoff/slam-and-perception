cmake_minimum_required(VERSION 3.14)
project(GtsamProject)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

include(FetchContent)

# ---------------------------------------------------------------------------
# System dependencies
# ---------------------------------------------------------------------------
find_package(GTSAM REQUIRED)
find_package(Protobuf REQUIRED)

# ---------------------------------------------------------------------------
# FetchContent dependencies
# ---------------------------------------------------------------------------

# MCAP C++ — header-only, no CMakeLists.txt in the repo
FetchContent_Declare(
    mcap
    GIT_REPOSITORY https://github.com/foxglove/mcap.git
    GIT_TAG        releases/cpp/v2.1.3
)
FetchContent_MakeAvailable(mcap)

add_library(mcap INTERFACE)
target_include_directories(mcap INTERFACE
    ${mcap_SOURCE_DIR}/cpp/mcap/include
)

# nlohmann/json
FetchContent_Declare(
    json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.3
)
FetchContent_MakeAvailable(json)

# small_gicp — header-only point cloud registration (ICP/GICP)
FetchContent_Declare(
    small_gicp
    GIT_REPOSITORY https://github.com/koide3/small_gicp.git
    GIT_TAG        v1.0.0
)
FetchContent_MakeAvailable(small_gicp)

# ---------------------------------------------------------------------------
# Sibling: slam-types (header-only)
# ---------------------------------------------------------------------------
add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../slam-types
    ${CMAKE_CURRENT_BINARY_DIR}/slam-types
    EXCLUDE_FROM_ALL
)

# ---------------------------------------------------------------------------
# Target: nuscenes_loader
# ---------------------------------------------------------------------------

# --- Proto generation from MCAP at build time ---
# The Python script extracts .proto from MCAP FileDescriptorSets, then runs
# protoc to produce .pb.h / .pb.cc.  A stamp file tracks completion.
set(MCAP_DATA_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../data/nuscenes/mcap)
set(MCAP_SAMPLE   ${MCAP_DATA_DIR}/NuScenes-v1.0-mini-scene-0061.mcap)
set(PROTO_DIR     ${CMAKE_CURRENT_BINARY_DIR}/schemas)
set(PROTO_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/proto-gen)
set(PROTO_STAMP   ${CMAKE_CURRENT_BINARY_DIR}/proto-gen.stamp)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

add_custom_command(
    OUTPUT ${PROTO_STAMP}
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/scripts/extract_and_compile_protos.py
            ${MCAP_SAMPLE}
            ${PROTO_DIR}
            ${PROTO_GEN_DIR}
            ${Protobuf_PROTOC_EXECUTABLE}
    COMMAND ${CMAKE_COMMAND} -E touch ${PROTO_STAMP}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/extract_and_compile_protos.py
    COMMENT "Extracting protobuf schemas from MCAP and compiling to C++"
    VERBATIM
)

add_custom_target(generate_protos DEPENDS ${PROTO_STAMP})

# Glob the generated .pb.cc after the custom command has run.
# On the first build the directory won't exist yet, so we list the known
# outputs explicitly.  These match the foxglove schema set in NuScenes MCAP.
set(_fg ${PROTO_GEN_DIR}/foxglove)
set(_gp ${PROTO_GEN_DIR}/google/protobuf)
set(PROTO_SRCS
    ${_fg}/ArrowPrimitive.pb.cc      ${_fg}/CameraCalibration.pb.cc
    ${_fg}/CircleAnnotation.pb.cc     ${_fg}/Color.pb.cc
    ${_fg}/CompressedImage.pb.cc      ${_fg}/CubePrimitive.pb.cc
    ${_fg}/CylinderPrimitive.pb.cc    ${_fg}/FrameTransform.pb.cc
    ${_fg}/Grid.pb.cc                 ${_fg}/ImageAnnotations.pb.cc
    ${_fg}/KeyValuePair.pb.cc         ${_fg}/LinePrimitive.pb.cc
    ${_fg}/LocationFix.pb.cc          ${_fg}/ModelPrimitive.pb.cc
    ${_fg}/PackedElementField.pb.cc   ${_fg}/Point2.pb.cc
    ${_fg}/Point3.pb.cc               ${_fg}/PointCloud.pb.cc
    ${_fg}/PointsAnnotation.pb.cc     ${_fg}/Pose.pb.cc
    ${_fg}/PoseInFrame.pb.cc          ${_fg}/Quaternion.pb.cc
    ${_fg}/SceneEntity.pb.cc          ${_fg}/SceneEntityDeletion.pb.cc
    ${_fg}/SceneUpdate.pb.cc          ${_fg}/SpherePrimitive.pb.cc
    ${_fg}/TextAnnotation.pb.cc       ${_fg}/TextPrimitive.pb.cc
    ${_fg}/TriangleListPrimitive.pb.cc ${_fg}/Vector2.pb.cc
    ${_fg}/Vector3.pb.cc
    ${_gp}/duration.pb.cc             ${_gp}/timestamp.pb.cc
)

# Foxglove-only protos (without google/protobuf/*.pb.cc) for the Python module.
# System libprotobuf already registers google/protobuf descriptors; including
# our own copies causes fatal double-registration when loaded alongside Python.
set(PROTO_SRCS_FOXGLOVE
    ${_fg}/ArrowPrimitive.pb.cc      ${_fg}/CameraCalibration.pb.cc
    ${_fg}/CircleAnnotation.pb.cc     ${_fg}/Color.pb.cc
    ${_fg}/CompressedImage.pb.cc      ${_fg}/CubePrimitive.pb.cc
    ${_fg}/CylinderPrimitive.pb.cc    ${_fg}/FrameTransform.pb.cc
    ${_fg}/Grid.pb.cc                 ${_fg}/ImageAnnotations.pb.cc
    ${_fg}/KeyValuePair.pb.cc         ${_fg}/LinePrimitive.pb.cc
    ${_fg}/LocationFix.pb.cc          ${_fg}/ModelPrimitive.pb.cc
    ${_fg}/PackedElementField.pb.cc   ${_fg}/Point2.pb.cc
    ${_fg}/Point3.pb.cc               ${_fg}/PointCloud.pb.cc
    ${_fg}/PointsAnnotation.pb.cc     ${_fg}/Pose.pb.cc
    ${_fg}/PoseInFrame.pb.cc          ${_fg}/Quaternion.pb.cc
    ${_fg}/SceneEntity.pb.cc          ${_fg}/SceneEntityDeletion.pb.cc
    ${_fg}/SceneUpdate.pb.cc          ${_fg}/SpherePrimitive.pb.cc
    ${_fg}/TextAnnotation.pb.cc       ${_fg}/TextPrimitive.pb.cc
    ${_fg}/TriangleListPrimitive.pb.cc ${_fg}/Vector2.pb.cc
    ${_fg}/Vector3.pb.cc
)

# Mark generated sources so CMake doesn't complain they're missing at
# configure time.
set_source_files_properties(${PROTO_SRCS} ${PROTO_SRCS_FOXGLOVE} PROPERTIES GENERATED TRUE)

# ---------------------------------------------------------------------------
# Target: nufuse (refactored pipeline)
# ---------------------------------------------------------------------------
add_executable(nufuse
    src/main_nufuse.cpp
    src/core/config_loader.cpp
    src/io/mcap_loader.cpp
    src/io/mcap_writer.cpp
    src/pipeline/pipeline.cpp
    src/processing/bias_estimator.cpp
    src/processing/forward_velocity_generator.cpp
    src/processing/geo_utils.cpp
    src/processing/gps_corruptor.cpp
    src/processing/lidar_odometry.cpp
    src/processing/measurement_merger.cpp
    src/processing/measurement_processor.cpp
    src/processing/nhc_generator.cpp
    src/processing/radar_processor.cpp
    src/graph/builder.cpp
    src/results/optimizer.cpp
    src/results/reporter.cpp
    ${PROTO_SRCS}
)

add_dependencies(nufuse generate_protos)

target_include_directories(nufuse PRIVATE
    ${PROTO_GEN_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(nufuse PRIVATE
    mcap
    nlohmann_json::nlohmann_json
    protobuf::libprotobuf
    slam_types
    gtsam
    small_gicp
    lz4
    zstd
)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
option(NUFUSE_BUILD_TESTS "Build factor unit tests" ON)

if(NUFUSE_BUILD_TESTS)
    enable_testing()

    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.15.2
    )
    set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    # --- Spatiotemporal Radar Factor ---
    add_executable(test_spatiotemporal_radar_factor
        tests/test_spatiotemporal_radar_factor.cpp
    )
    target_include_directories(test_spatiotemporal_radar_factor PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    target_link_libraries(test_spatiotemporal_radar_factor PRIVATE
        gtsam
        GTest::gtest_main
    )
    add_test(NAME test_spatiotemporal_radar_factor COMMAND test_spatiotemporal_radar_factor)

    # --- Lidar Extrinsics Factor ---
    add_executable(test_lidar_extrinsics_factor
        tests/test_lidar_extrinsics_factor.cpp
    )
    target_include_directories(test_lidar_extrinsics_factor PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    target_link_libraries(test_lidar_extrinsics_factor PRIVATE
        gtsam
        GTest::gtest_main
    )
    add_test(NAME test_lidar_extrinsics_factor COMMAND test_lidar_extrinsics_factor)

    # --- Geo Utils ---
    add_executable(test_geo_utils
        tests/test_geo_utils.cpp
        src/processing/geo_utils.cpp
    )
    target_include_directories(test_geo_utils PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    target_link_libraries(test_geo_utils PRIVATE
        slam_types
        gtsam
        GTest::gtest_main
    )
    add_test(NAME test_geo_utils COMMAND test_geo_utils)

    # --- Config Loader ---
    add_executable(test_config_loader
        tests/test_config_loader.cpp
        src/core/config_loader.cpp
    )
    target_include_directories(test_config_loader PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    target_link_libraries(test_config_loader PRIVATE
        slam_types
        nlohmann_json::nlohmann_json
        gtsam
        GTest::gtest_main
    )
    add_test(NAME test_config_loader COMMAND test_config_loader)
endif()

# ---------------------------------------------------------------------------
# Python bindings (pybind11)
# ---------------------------------------------------------------------------
option(NUFUSE_BUILD_PYTHON "Build Python bindings" ON)

if(NUFUSE_BUILD_PYTHON)
    find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

    # pybind11 ships with GTSAM's wrapper toolkit
    set(pybind11_DIR "/usr/local/lib/gtwrap/pybind11")
    add_subdirectory(${pybind11_DIR} ${CMAKE_CURRENT_BINARY_DIR}/pybind11 EXCLUDE_FROM_ALL)

    pybind11_add_module(_nufuse
        src/bindings/pynufuse.cpp
        src/core/config_loader.cpp
        src/io/mcap_loader.cpp
        src/pipeline/pipeline.cpp
        src/processing/bias_estimator.cpp
        src/processing/forward_velocity_generator.cpp
        src/processing/geo_utils.cpp
        src/processing/gps_corruptor.cpp
        src/processing/lidar_odometry.cpp
        src/processing/measurement_merger.cpp
        src/processing/measurement_processor.cpp
        src/processing/nhc_generator.cpp
        src/processing/radar_processor.cpp
        src/graph/builder.cpp
        src/results/optimizer.cpp
        src/results/reporter.cpp
        ${PROTO_SRCS_FOXGLOVE}
    )

    add_dependencies(_nufuse generate_protos)

    target_include_directories(_nufuse PRIVATE
        ${PROTO_GEN_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )

    target_link_libraries(_nufuse PRIVATE
        mcap
        nlohmann_json::nlohmann_json
        protobuf::libprotobuf
        slam_types
        gtsam
        small_gicp
        lz4
        zstd
    )

    # Hide protobuf symbols to avoid descriptor pool clashes with Python protobuf
    target_link_options(_nufuse PRIVATE
        "LINKER:--exclude-libs,ALL"
    )
    target_compile_definitions(_nufuse PRIVATE
        PROTOBUF_USE_DLLS
    )

    # Install into the source tree for easy import
    set_target_properties(_nufuse PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/nufuse"
    )
endif()

