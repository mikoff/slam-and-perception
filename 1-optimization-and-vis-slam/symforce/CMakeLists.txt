cmake_minimum_required(VERSION 3.14)
project(SymforceProject)

set(CMAKE_CXX_STANDARD 21)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
  eigen 
  URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
)
FetchContent_MakeAvailable(eigen)

add_executable(my_app src/main.cpp)
target_link_libraries(my_app PRIVATE Eigen3::Eigen)