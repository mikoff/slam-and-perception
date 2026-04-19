/// @file symforce_traits.hpp
/// @brief GeometryTraits specialization for symforce's sym::Pose2<Scalar>.
///
/// Include this header when using safe_geometry.hpp with sym::Pose2d (or any
/// sym::Pose2<Scalar> instantiation).  The default traits assume lowercase
/// `inverse()`, but symforce uses `Inverse()`.
///
/// @code
/// #include <slam/geometry/safe_geometry.hpp>
/// #include <slam/geometry/traits/symforce_traits.hpp>
/// @endcode

#pragma once

#include <slam/geometry/safe_geometry.hpp>
#include <sym/pose2.h>

namespace slam::geometry {

template <typename Scalar>
struct GeometryTraits<sym::Pose2<Scalar>> {
    using Pose = sym::Pose2<Scalar>;
    using Vector2 = Eigen::Matrix<Scalar, 2, 1>;

    [[nodiscard]] static Pose inverse(const Pose& val) {
        return val.Inverse();
    }

    [[nodiscard]] static Pose compose(const Pose& lhs, const Pose& rhs) {
        return lhs * rhs;
    }

    template <typename PointT>
    [[nodiscard]] static PointT action(const Pose& pose, const PointT& point) {
        return pose * point;
    }
};

} // namespace slam::geometry
