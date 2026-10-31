#ifndef HEXAPOD_CORE_MATH_TRANSFORM_3D_HPP_
#define HEXAPOD_CORE_MATH_TRANSFORM_3D_HPP_

#include <cmath>

#include "hexapod_core/math/point_3d.hpp"

namespace hexapod::core {

// Represents a 3D rigid body transformation with planar yaw rotation and 3D translation.
// Optimized for mounting poses of legs on a planar hexapod chassis.
class Transform3D {
public:
    constexpr Transform3D() noexcept = default;
    Transform3D(const Point3D& translation, double yawRad) noexcept
        : translation_(translation), yawRad_(yawRad), cosYaw_(std::cos(yawRad)), sinYaw_(std::sin(yawRad)) {}

    [[nodiscard]] const Point3D& translation() const noexcept { return translation_; }
    [[nodiscard]] double yaw() const noexcept { return yawRad_; }

    // Transforms a point from local frame to parent/body frame: p_parent = R_z(yaw) * p_local + translation
    [[nodiscard]] Point3D transformPoint(const Point3D& localPoint) const noexcept {
        return Point3D{
            translation_.x + localPoint.x * cosYaw_ - localPoint.y * sinYaw_,
            translation_.y + localPoint.x * sinYaw_ + localPoint.y * cosYaw_,
            translation_.z + localPoint.z
        };
    }

    // Transforms a point from parent/body frame to local frame: p_local = R_z(-yaw) * (p_parent - translation)
    [[nodiscard]] Point3D inverseTransformPoint(const Point3D& parentPoint) const noexcept {
        const double dx = parentPoint.x - translation_.x;
        const double dy = parentPoint.y - translation_.y;
        const double dz = parentPoint.z - translation_.z;
        return Point3D{
            dx * cosYaw_ + dy * sinYaw_,
            -dx * sinYaw_ + dy * cosYaw_,
            dz
        };
    }

private:
    Point3D translation_{};
    double yawRad_{0.0};
    double cosYaw_{1.0};
    double sinYaw_{0.0};
};

}  // namespace hexapod::core

#endif  // HEXAPOD_CORE_MATH_TRANSFORM_3D_HPP_

