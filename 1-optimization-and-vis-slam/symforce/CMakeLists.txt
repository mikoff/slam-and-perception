cmake_minimum_required(VERSION 3.14)
project(Week1Symforce)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Fetch SymForce from source
include(FetchContent)
FetchContent_Declare(
  symforce
  GIT_REPOSITORY https://github.com/symforce-org/symforce.git
  GIT_TAG v0.10.1
)
FetchContent_MakeAvailable(symforce)

add_executable(my_app src/main.cpp)

# Link the SymForce optimization library
target_link_libraries(my_app PRIVATE symforce_opt)