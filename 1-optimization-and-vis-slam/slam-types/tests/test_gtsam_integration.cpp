/// @file test_gtsam_integration.cpp
/// @brief Integration tests proving safe_geometry and strong_id work with GTSAM.

#include <slam/core/strong_id.hpp>
#include <slam/geometry/safe_geometry.hpp>

#include <gtest/gtest.h>

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>

#include <cmath>
#include <numbers>
#include <unordered_map>
#include <vector>

// ---- Frame tags -----------------------------------------------------------
struct MapFrame {};
struct VehicleFrame {};
struct SensorFrame {};

// ---- Domain aliases -------------------------------------------------------
using Pose2_MV      = slam::geometry::Pose<gtsam::Pose2, VehicleFrame, MapFrame>;
using Transform2_VS = slam::geometry::Transform<gtsam::Pose2, VehicleFrame, SensorFrame>;
using Point2_V      = slam::geometry::TaggedPoint<gtsam::Point2, VehicleFrame>;

// ---- StrongId tags --------------------------------------------------------
struct PoseTag {};
struct LandmarkTag {};
using PoseId     = slam::core::StrongId<PoseTag>;
using LandmarkId = slam::core::StrongId<LandmarkTag>;

// ---------------------------------------------------------------------------
// Geometry integration
// ---------------------------------------------------------------------------

TEST(GtsamIntegration, PoseComposition) {
    Pose2_MV T_map_veh(gtsam::Pose2(5.0, 0.0, 0.0));
    Transform2_VS T_veh_sens(gtsam::Pose2(1.0, 0.0, 0.0));

    auto T_map_sens = T_map_veh * T_veh_sens;

    EXPECT_NEAR(T_map_sens.value().x(), 6.0, 1e-9);
    EXPECT_NEAR(T_map_sens.value().y(), 0.0, 1e-9);
}

TEST(GtsamIntegration, PointAction) {
    Pose2_MV T_map_veh(gtsam::Pose2(10.0, 0.0, std::numbers::pi / 2.0));
    Point2_V p_veh(gtsam::Point2(5.0, 0.0));

    auto p_map = T_map_veh * p_veh;

    EXPECT_NEAR(p_map.value().x(), 10.0, 1e-6);
    EXPECT_NEAR(p_map.value().y(), 5.0, 1e-6);
}

TEST(GtsamIntegration, BetweenAndInverse) {
    Pose2_MV T_map_vehA(gtsam::Pose2(2.0, 0.0, 0.0));
    Pose2_MV T_map_vehB(gtsam::Pose2(10.0, 0.0, 0.0));

    auto T_vehA_vehB = slam::geometry::between(T_map_vehA, T_map_vehB);

    EXPECT_NEAR(T_vehA_vehB.value().x(), 8.0, 1e-9);
    EXPECT_NEAR(T_vehA_vehB.value().y(), 0.0, 1e-9);
}

TEST(GtsamIntegration, StdVectorOfPoses) {
    std::vector<Pose2_MV> trajectory(10);
    EXPECT_EQ(trajectory.size(), 10u);

    gtsam::Pose2 raw(1.0, 2.0, 3.14);
    Pose2_MV pose(std::move(raw));
    trajectory[0] = pose;
    EXPECT_NEAR(trajectory[0].value().x(), 1.0, 1e-9);
}

// ---------------------------------------------------------------------------
// StrongId integration — typed pose graph storage
// ---------------------------------------------------------------------------

TEST(GtsamIntegration, TypedPoseGraph) {
    std::unordered_map<PoseId, gtsam::Pose2> poses;
    poses[PoseId(0)] = gtsam::Pose2(0.0, 0.0, 0.0);
    poses[PoseId(1)] = gtsam::Pose2(2.0, 0.0, 0.0);

    EXPECT_EQ(poses.size(), 2u);
    EXPECT_NEAR(poses.at(PoseId(1)).x(), 2.0, 1e-9);
}

TEST(GtsamIntegration, MixedIdTypeSafety) {
    // PoseId and LandmarkId share the same raw value but are distinct types.
    PoseId pid(42);
    LandmarkId lid(42);

    EXPECT_FALSE((std::is_same_v<decltype(pid), decltype(lid)>));
    EXPECT_EQ(pid.value(), lid.value()); // same raw value, different types
}

// ---------------------------------------------------------------------------
// Full pipeline: StrongId + SafeGeometry
// ---------------------------------------------------------------------------

TEST(GtsamIntegration, FullPipeline) {
    // Simulate a mini pose graph: two poses, compute relative transform.
    std::unordered_map<PoseId, Pose2_MV> graph;
    graph[PoseId(0)] = Pose2_MV(gtsam::Pose2(0.0, 0.0, 0.0));
    graph[PoseId(1)] = Pose2_MV(gtsam::Pose2(5.0, 0.0, std::numbers::pi / 4.0));

    auto rel = slam::geometry::between(graph.at(PoseId(0)), graph.at(PoseId(1)));
    EXPECT_NEAR(rel.value().x(), 5.0, 1e-9);
}
