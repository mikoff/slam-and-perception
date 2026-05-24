/// @file core/frames.hpp
/// @brief Coordinate frame tags for compile-time frame safety.

#pragma once

namespace nufuse::cf {

struct Wgs84 {};
struct Enu {};
struct Map {};
struct Body {};
struct Imu {};

// Lidar
struct LidarTop {};

// Cameras
struct CamFront {};
struct CamFrontLeft {};
struct CamFrontRight {};
struct CamBack {};
struct CamBackLeft {};
struct CamBackRight {};

// Radars
struct RadarFront {};
struct RadarFrontLeft {};
struct RadarFrontRight {};
struct RadarBackLeft {};
struct RadarBackRight {};

}  // namespace nufuse::cf
