cmake_minimum_required(VERSION 3.14)
project(GtsamProject)
set(CMAKE_CXX_STANDARD 20)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(GTSAM REQUIRED)

add_executable(gtsam_example src/main.cpp)
target_link_libraries(gtsam_example PRIVATE gtsam)

