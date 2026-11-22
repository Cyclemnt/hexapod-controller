#ifndef HEXAPOD_CORE_LOCOMOTION_BODY_POSE_COMMAND_HPP_
#define HEXAPOD_CORE_LOCOMOTION_BODY_POSE_COMMAND_HPP_

#include <cmath>

#include "hexapod_core/math/point_3d.hpp"

namespace hexapod::core {

struct BodyPoseCommand {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double roll{0.0};
    double pitch{0.0};
    double yaw{0.0};

    constexpr bool operator==(const BodyPoseCommand& other) const noexcept = default;

    // Transforms a foot coordinate in world/nominal frame into the chassis/body frame
    [[nodiscard]] Point3D applyBodyIk(const Point3D& worldFoot) const noexcept {
        const double dx = worldFoot.x - x;
        const double dy = worldFoot.y - y;
        const double dz = worldFoot.z - z;

        // R_z(-yaw)
        const double cosYaw = std::cos(-yaw);
        const double sinYaw = std::sin(-yaw);
        const double x1 = dx * cosYaw - dy * sinYaw;
        const double y1 = dx * sinYaw + dy * cosYaw;
        const double z1 = dz;

        // R_y(-pitch)
        const double cosPitch = std::cos(-pitch);
        const double sinPitch = std::sin(-pitch);
        const double x2 = x1 * cosPitch + z1 * sinPitch;
        const double y2 = y1;
        const double z2 = -x1 * sinPitch + z1 * cosPitch;

        // R_x(-roll)
        const double cosRoll = std::cos(-roll);
        const double sinRoll = std::sin(-roll);
        const double x3 = x2;
        const double y3 = y2 * cosRoll - z2 * sinRoll;
        const double z3 = y2 * sinRoll + z2 * cosRoll;

        return Point3D{x3, y3, z3};
    }

    // Transforms a foot coordinate from chassis/body frame back into world/nominal frame
    // (exact inverse of applyBodyIk)
    [[nodiscard]] Point3D applyBodyFk(const Point3D& bodyFoot) const noexcept {
        // R_x(+roll)
        const double cosRoll = std::cos(roll);
        const double sinRoll = std::sin(roll);
        const double x1 = bodyFoot.x;
        const double y1 = bodyFoot.y * cosRoll - bodyFoot.z * sinRoll;
        const double z1 = bodyFoot.y * sinRoll + bodyFoot.z * cosRoll;

        // R_y(+pitch)
        const double cosPitch = std::cos(pitch);
        const double sinPitch = std::sin(pitch);
        const double x2 = x1 * cosPitch + z1 * sinPitch;
        const double y2 = y1;
        const double z2 = -x1 * sinPitch + z1 * cosPitch;

        // R_z(+yaw)
        const double cosYaw = std::cos(yaw);
        const double sinYaw = std::sin(yaw);
        const double x3 = x2 * cosYaw - y2 * sinYaw;
        const double y3 = x2 * sinYaw + y2 * cosYaw;
        const double z3 = z2;

        return Point3D{x3 + x, y3 + y, z3 + z};
    }
};

}  // namespace hexapod::core

#endif  // HEXAPOD_CORE_LOCOMOTION_BODY_POSE_COMMAND_HPP_
