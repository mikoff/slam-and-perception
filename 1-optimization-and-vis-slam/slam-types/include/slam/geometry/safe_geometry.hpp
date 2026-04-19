/// @file safe_geometry.hpp
/// @brief Compile-time safe geometry wrappers using Tagged Types and Inheritance.
///
/// Provides Transform, Pose, and TaggedPoint templates that enforce coordinate
/// frame consistency at compile time via phantom frame-tag types.

#pragma once

#include <type_traits>
#include <utility>

namespace slam::geometry {

// ============================================================================
// 1. Backend Math Traits
// ============================================================================

/// @brief Default geometry traits — assumes the backend type provides
///        `inverse()`, `operator*(T)`, and `operator*(PointT)`.
///
/// Specialize this struct for backends with a different API
/// (e.g. symforce uses `Inverse()` instead of `inverse()`).
template <typename T>
struct GeometryTraits {
    [[nodiscard]] static constexpr T inverse(const T& val) {
        return val.inverse();
    }

    [[nodiscard]] static constexpr T compose(const T& lhs, const T& rhs) {
        return lhs * rhs;
    }

    template <typename PointT>
    [[nodiscard]] static constexpr PointT action(const T& pose, const PointT& point) {
        return pose * point;
    }
};

// ============================================================================
// 2. Core Wrappers
// ============================================================================

/// @brief A point tagged with its coordinate frame.
template <typename DataT, typename Frame>
class TaggedPoint {
private:
    DataT point_data;

public:
    constexpr TaggedPoint() = default;

    explicit constexpr TaggedPoint(DataT val) : point_data(std::move(val)) {}

    [[nodiscard]] constexpr const DataT& value() const noexcept { return point_data; }
};

/// @brief A transformation mapping points FROM the Source TO the Target frame.
///
/// Lie Group Foundation: represents an element of SE(2) or SE(3) that actively
/// moves data between coordinate frames.
template <typename DataT, typename TargetFrame, typename SourceFrame>
class Transform {
protected:
    DataT pose_data;

public:
    constexpr Transform() = default;

    explicit constexpr Transform(DataT val) : pose_data(std::move(val)) {}

    [[nodiscard]] constexpr const DataT& value() const noexcept { return pose_data; }

    [[nodiscard]] constexpr Transform<DataT, SourceFrame, TargetFrame> inverse() const {
        return Transform<DataT, SourceFrame, TargetFrame>(
            GeometryTraits<DataT>::inverse(pose_data)
        );
    }
};

// ============================================================================
// 3. Semantic Derived Class
// ============================================================================

/// @brief The spatial state of an Entity relative to a Reference Frame.
///
/// State/Action Duality: a Pose of Entity in Reference IS the Transform
/// from Entity to Reference (T_{Reference←Entity}).
template <typename DataT, typename EntityFrame, typename ReferenceFrame>
class Pose : public Transform<DataT, ReferenceFrame, EntityFrame> {
public:
    using Transform<DataT, ReferenceFrame, EntityFrame>::Transform;
};

// ============================================================================
// 4. Safe Operators
// ============================================================================

/// @brief Compose two transformations with compile-time frame cancellation.
///
/// @param lhs The left-hand transformation ($T_{A \leftarrow B}$).
/// @param rhs The right-hand transformation ($T_{B \leftarrow C}$).
/// @return A new transformation mapping from C to A ($T_{A \leftarrow C}$).
template <typename DataT, typename FA, typename FB, typename FC, typename FD>
[[nodiscard]] constexpr auto operator*(const Transform<DataT, FA, FB>& lhs,
                                       const Transform<DataT, FC, FD>& rhs)
{
    static_assert(std::is_same_v<FB, FC>,
        "\n\n[GEOMETRY ERROR]: Invalid Pose Composition!\n"
        "The Source frame of the left operand must match the Target frame of the right operand.\n"
        "Expected: T_A_B * T_B_C\n\n");

    return Transform<DataT, FA, FD>(GeometryTraits<DataT>::compose(lhs.value(), rhs.value()));
}

/// @brief Project a point from its local frame into a new coordinate frame.
/// @note The compilation will fail if the point's coordinate frame does not 
/// match the Source frame of the applied transformation.
/// @param tf The transformation matrix ($T_{A \leftarrow B}$).
/// @param pt The point residing in the source frame ($p_B$).
/// @return A new point projected into the target frame ($p_A$).
template <typename PoseT, typename PointT, typename FA, typename FB, typename FC>
[[nodiscard]] constexpr auto operator*(const Transform<PoseT, FA, FB>& tf,
                                       const TaggedPoint<PointT, FC>& pt)
{
    static_assert(std::is_same_v<FB, FC>,
        "\n\n[GEOMETRY ERROR]: Invalid Transformation!\n"
        "The Source frame of the Transform must match the Frame of the Point.\n"
        "Expected: T_A_B * p_B\n\n");

    return TaggedPoint<PointT, FA>(GeometryTraits<PoseT>::template action<PointT>(tf.value(), pt.value()));
}

/// @brief Compute the relative transform between two poses sharing a reference frame.
///
/// between(T_{Ref←A}, T_{Ref←B}) = T_{A←B}
template <typename DataT, typename EntityA, typename EntityB, typename Reference>
[[nodiscard]] constexpr auto between(const Pose<DataT, EntityA, Reference>& poseA,
                                     const Pose<DataT, EntityB, Reference>& poseB)
{
    return poseA.inverse() * poseB;
}

} // namespace slam::geometry
