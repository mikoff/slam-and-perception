cmake_minimum_required(VERSION 3.10)
project(MyHybridApp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(my_app src/main.cpp)
