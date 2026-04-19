cmake_minimum_required(VERSION 3.14)
project(SlamTypes VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ---------------------------------------------------------------------------
# Header-only INTERFACE library
# ---------------------------------------------------------------------------
add_library(slam_types INTERFACE)
target_include_directories(slam_types INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# ---------------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------------
option(SLAM_TYPES_BUILD_TESTS    "Build unit tests"                 ON)
option(SLAM_TYPES_TEST_GTSAM     "Build GTSAM integration tests"    OFF)
option(SLAM_TYPES_TEST_SYMFORCE  "Build SymForce integration tests" OFF)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(SLAM_TYPES_BUILD_TESTS)
    enable_testing()
    include(FetchContent)

    # GoogleTest
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.15.2
    )
    set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    # ---- Core unit tests (zero external deps) ----
    add_executable(test_strong_id tests/test_strong_id.cpp)
    target_link_libraries(test_strong_id PRIVATE slam_types GTest::gtest_main)
    add_test(NAME test_strong_id COMMAND test_strong_id)

    add_executable(test_safe_geometry tests/test_safe_geometry.cpp)
    target_link_libraries(test_safe_geometry PRIVATE slam_types GTest::gtest_main)
    add_test(NAME test_safe_geometry COMMAND test_safe_geometry)

    # ---- GTSAM integration tests ----
    if(SLAM_TYPES_TEST_GTSAM)
        find_package(GTSAM REQUIRED)

        add_executable(test_gtsam_integration tests/test_gtsam_integration.cpp)
        target_link_libraries(test_gtsam_integration PRIVATE slam_types gtsam GTest::gtest_main)
        add_test(NAME test_gtsam_integration COMMAND test_gtsam_integration)
    endif()

    # ---- SymForce integration tests ----
    # Uses a pre-built symforce from the sibling project (avoids re-fetching
    # and the complex Python build dependencies).
    if(SLAM_TYPES_TEST_SYMFORCE)
        set(SYMFORCE_PROJECT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../symforce" CACHE PATH
            "Path to the symforce sub-project (with a completed build/)")

        set(_sf_src "${SYMFORCE_PROJECT_DIR}/build/_deps/symforce-src")
        set(_sf_lib "${SYMFORCE_PROJECT_DIR}/build/_deps/symforce-build/symforce/opt")
        set(_sf_eigen "${SYMFORCE_PROJECT_DIR}/build/_deps/eigen3-src")

        if(EXISTS "${_sf_src}/gen/cpp/sym/pose2.h" AND EXISTS "${_sf_lib}/libsymforce_opt.so")
            # Imported target for the pre-built shared library
            add_library(symforce_gen_imported SHARED IMPORTED)
            set_target_properties(symforce_gen_imported PROPERTIES
                IMPORTED_LOCATION "${SYMFORCE_PROJECT_DIR}/build/_deps/symforce-build/libsymforce_gen.so"
            )
            add_library(symforce_opt_imported SHARED IMPORTED)
            set_target_properties(symforce_opt_imported PROPERTIES
                IMPORTED_LOCATION "${_sf_lib}/libsymforce_opt.so"
            )

            add_executable(test_symforce_integration tests/test_symforce_integration.cpp)
            target_link_libraries(test_symforce_integration PRIVATE
                slam_types symforce_opt_imported symforce_gen_imported GTest::gtest_main)
            target_include_directories(test_symforce_integration PRIVATE
                "${SYMFORCE_PROJECT_DIR}/build/_deps/symforce-build/lcmtypes/cpp"
                "${SYMFORCE_PROJECT_DIR}/build/_deps/symforce-src/third_party/skymarshal/include"
                "${_sf_src}/gen/cpp"
                "${_sf_src}"
                "${_sf_eigen}"
            )
            add_test(NAME test_symforce_integration COMMAND test_symforce_integration)
        else()
            message(WARNING
                "SymForce pre-built artifacts not found at ${SYMFORCE_PROJECT_DIR}/build/. "
                "Build the symforce project first, or set SLAM_TYPES_TEST_SYMFORCE=OFF.")
        endif()
    endif()
endif()
