/// @file test_symforce_integration.cpp
/// @brief Integration tests proving safe_geometry and strong_id work with SymForce.

#include <slam/core/strong_id.hpp>
#include <slam/geometry/safe_geometry.hpp>
#include <slam/geometry/traits/symforce_traits.hpp>

#include <gtest/gtest.h>

#include <sym/pose2.h>

#include <cmath>
#include <numbers>
#include <unordered_map>
#include <vector>

// ---- Frame tags -----------------------------------------------------------
struct MapFrame {};
struct VehicleFrame {};
struct SensorFrame {};

// ---- Domain aliases (sym::Pose2d = sym::Pose2<double>) --------------------
using Pose2_MV      = slam::geometry::Pose<sym::Pose2d, VehicleFrame, MapFrame>;
using Transform2_VS = slam::geometry::Transform<sym::Pose2d, VehicleFrame, SensorFrame>;
using Point2_V      = slam::geometry::TaggedPoint<Eigen::Vector2d, VehicleFrame>;

// ---- StrongId tags --------------------------------------------------------
struct PoseTag {};
using PoseId = slam::core::StrongId<PoseTag>;

// Helper to build a sym::Pose2d from (x, y, theta)
static sym::Pose2d make_pose(double x, double y, double theta) {
    return sym::Pose2d(sym::Rot2d::FromAngle(theta), Eigen::Vector2d(x, y));
}

// ---------------------------------------------------------------------------
// Geometry integration
// ---------------------------------------------------------------------------

TEST(SymforceIntegration, PoseComposition) {
    Pose2_MV T_map_veh(make_pose(5.0, 0.0, 0.0));
    Transform2_VS T_veh_sens(make_pose(1.0, 0.0, 0.0));

    auto T_map_sens = T_map_veh * T_veh_sens;

    EXPECT_NEAR(T_map_sens.value().Position().x(), 6.0, 1e-9);
    EXPECT_NEAR(T_map_sens.value().Position().y(), 0.0, 1e-9);
}

TEST(SymforceIntegration, PointAction) {
    Pose2_MV T_map_veh(make_pose(10.0, 0.0, std::numbers::pi / 2.0));
    Point2_V p_veh(Eigen::Vector2d(5.0, 0.0));

    auto p_map = T_map_veh * p_veh;

    EXPECT_NEAR(p_map.value().x(), 10.0, 1e-6);
    EXPECT_NEAR(p_map.value().y(), 5.0, 1e-6);
}

TEST(SymforceIntegration, InverseRoundTrip) {
    Pose2_MV pose(make_pose(3.0, 4.0, 0.5));
    auto inv = pose.inverse();
    auto identity = pose * inv;

    EXPECT_NEAR(identity.value().Position().x(), 0.0, 1e-9);
    EXPECT_NEAR(identity.value().Position().y(), 0.0, 1e-9);
}

TEST(SymforceIntegration, Between) {
    Pose2_MV T_map_vehA(make_pose(2.0, 0.0, 0.0));
    Pose2_MV T_map_vehB(make_pose(10.0, 0.0, 0.0));

    auto T_vehA_vehB = slam::geometry::between(T_map_vehA, T_map_vehB);

    EXPECT_NEAR(T_vehA_vehB.value().Position().x(), 8.0, 1e-9);
    EXPECT_NEAR(T_vehA_vehB.value().Position().y(), 0.0, 1e-9);
}

TEST(SymforceIntegration, StdVectorOfPoses) {
    std::vector<Pose2_MV> trajectory(5);
    EXPECT_EQ(trajectory.size(), 5u);

    trajectory[0] = Pose2_MV(make_pose(1.0, 2.0, 0.0));
    EXPECT_NEAR(trajectory[0].value().Position().x(), 1.0, 1e-9);
}

// ---------------------------------------------------------------------------
// StrongId + SafeGeometry full pipeline
// ---------------------------------------------------------------------------

TEST(SymforceIntegration, FullPipeline) {
    std::unordered_map<PoseId, Pose2_MV> graph;
    graph[PoseId(0)] = Pose2_MV(make_pose(0.0, 0.0, 0.0));
    graph[PoseId(1)] = Pose2_MV(make_pose(5.0, 0.0, std::numbers::pi / 4.0));

    auto rel = slam::geometry::between(graph.at(PoseId(0)), graph.at(PoseId(1)));
    EXPECT_NEAR(rel.value().Position().x(), 5.0, 1e-9);
}
