/// @file test_safe_geometry.cpp
/// @brief GTest unit tests for slam::geometry templates using a minimal mock backend.

#include <slam/geometry/safe_geometry.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace {

// ---------------------------------------------------------------------------
// Minimal 2-D translation group (R², +) — satisfies GeometryTraits' default
// contract (inverse(), operator*) without external dependencies.
// ---------------------------------------------------------------------------
struct Trans2D {
    double x = 0;
    double y = 0;

    Trans2D() = default;
    Trans2D(double x, double y) : x(x), y(y) {}

    Trans2D inverse() const { return {-x, -y}; }
    Trans2D operator*(const Trans2D& o) const { return {x + o.x, y + o.y}; }
    bool operator==(const Trans2D& o) const {
        return std::abs(x - o.x) < 1e-9 && std::abs(y - o.y) < 1e-9;
    }
};

// Frame tags
struct World {};
struct Body {};
struct Sensor {};
struct Camera {};

using Tf_W_B = slam::geometry::Transform<Trans2D, World, Body>;
using Tf_B_S = slam::geometry::Transform<Trans2D, Body, Sensor>;
using Pose_B_W = slam::geometry::Pose<Trans2D, Body, World>;
using Point_B  = slam::geometry::TaggedPoint<Trans2D, Body>;

} // namespace

// ---- Transform Construction & Value Access --------------------------------

TEST(SafeGeometry, TransformDefaultConstructs) {
    Tf_W_B tf;
    EXPECT_DOUBLE_EQ(tf.value().x, 0.0);
    EXPECT_DOUBLE_EQ(tf.value().y, 0.0);
}

TEST(SafeGeometry, TransformExplicitConstruction) {
    Tf_W_B tf(Trans2D(3.0, 4.0));
    EXPECT_DOUBLE_EQ(tf.value().x, 3.0);
    EXPECT_DOUBLE_EQ(tf.value().y, 4.0);
}

// ---- Transform Inverse ----------------------------------------------------

TEST(SafeGeometry, InverseNegatesTranslation) {
    Tf_W_B tf(Trans2D(5.0, -2.0));
    auto inv = tf.inverse(); // Transform<Trans2D, Body, World>
    EXPECT_DOUBLE_EQ(inv.value().x, -5.0);
    EXPECT_DOUBLE_EQ(inv.value().y, 2.0);
}

// ---- Composition (frame cancellation) -------------------------------------

TEST(SafeGeometry, CompositionCancelsFrames) {
    Tf_W_B tf_w_b(Trans2D(1.0, 0.0));
    Tf_B_S tf_b_s(Trans2D(0.0, 2.0));

    auto tf_w_s = tf_w_b * tf_b_s; // T_{World←Sensor}
    EXPECT_DOUBLE_EQ(tf_w_s.value().x, 1.0);
    EXPECT_DOUBLE_EQ(tf_w_s.value().y, 2.0);
}

// ---- Pose Inherits Transform ----------------------------------------------

TEST(SafeGeometry, PoseIsATransform) {
    Pose_B_W pose(Trans2D(10.0, 20.0));
    // Pose<Trans2D, Body, World> IS-A Transform<Trans2D, World, Body>
    EXPECT_DOUBLE_EQ(pose.value().x, 10.0);
}

TEST(SafeGeometry, PoseInverseWorks) {
    Pose_B_W pose(Trans2D(3.0, 4.0));
    auto inv = pose.inverse();
    EXPECT_DOUBLE_EQ(inv.value().x, -3.0);
    EXPECT_DOUBLE_EQ(inv.value().y, -4.0);
}

// ---- TaggedPoint -----------------------------------------------------------

TEST(SafeGeometry, TaggedPointDefaultConstructs) {
    Point_B pt;
    EXPECT_DOUBLE_EQ(pt.value().x, 0.0);
}

TEST(SafeGeometry, TaggedPointExplicitConstruction) {
    Point_B pt(Trans2D(1.0, 2.0));
    EXPECT_DOUBLE_EQ(pt.value().x, 1.0);
    EXPECT_DOUBLE_EQ(pt.value().y, 2.0);
}

// ---- Point Action ----------------------------------------------------------

TEST(SafeGeometry, PointActionTransformsFrame) {
    // T_{World←Body} applied to p_Body → p_World
    slam::geometry::Transform<Trans2D, World, Body> tf(Trans2D(10.0, 0.0));
    slam::geometry::TaggedPoint<Trans2D, Body> p_body(Trans2D(1.0, 2.0));

    auto p_world = tf * p_body;
    EXPECT_DOUBLE_EQ(p_world.value().x, 11.0);
    EXPECT_DOUBLE_EQ(p_world.value().y, 2.0);
}

// ---- Between ---------------------------------------------------------------

TEST(SafeGeometry, BetweenComputesRelative) {
    using PoseA = slam::geometry::Pose<Trans2D, Body, World>;
    using PoseB = slam::geometry::Pose<Trans2D, Sensor, World>;

    PoseA a(Trans2D(2.0, 0.0));
    PoseB b(Trans2D(10.0, 0.0));

    auto rel = slam::geometry::between(a, b); // T_{Body←Sensor}
    EXPECT_DOUBLE_EQ(rel.value().x, 8.0);
    EXPECT_DOUBLE_EQ(rel.value().y, 0.0);
}

// ---- STL Container Compatibility ------------------------------------------

TEST(SafeGeometry, TransformInVector) {
    std::vector<Pose_B_W> trajectory(5);
    EXPECT_EQ(trajectory.size(), 5u);
    trajectory[0] = Pose_B_W(Trans2D(1.0, 2.0));
    EXPECT_DOUBLE_EQ(trajectory[0].value().x, 1.0);
}

// ---- Move Semantics -------------------------------------------------------

TEST(SafeGeometry, MoveConstructionWorks) {
    Trans2D raw(7.0, 8.0);
    Tf_W_B tf(std::move(raw));
    EXPECT_DOUBLE_EQ(tf.value().x, 7.0);
}
