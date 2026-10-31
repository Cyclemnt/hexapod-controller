#ifndef HEXAPOD_CORE_ROBOT_LEG_MOUNTING_HPP_
#define HEXAPOD_CORE_ROBOT_LEG_MOUNTING_HPP_

#include "hexapod_core/math/point_3d.hpp"
#include "hexapod_core/math/transform_3d.hpp"

namespace hexapod::core {

struct LegMounting {
    Point3D originBodyFrame{};  // Vertex coordinate on the chassis in body frame (meters)
    double yawRad{0.0};         // Coxa baseline angle in body frame (radians)

    [[nodiscard]] Transform3D getTransform() const noexcept {
        return Transform3D(originBodyFrame, yawRad);
    }
};

}  // namespace hexapod::core

#endif  // HEXAPOD_CORE_ROBOT_LEG_MOUNTING_HPP_

