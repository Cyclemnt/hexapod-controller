#include "hexapod_core/robot/hexapod_geometry.hpp"

#include <cmath>

namespace hexapod::core {

namespace {

LegMounting createMountingFromAngle(double angleRad) noexcept {
    const double x = HexapodGeometry::kChassisRadius * std::cos(angleRad);
    const double y = HexapodGeometry::kChassisRadius * std::sin(angleRad);
    return LegMounting{.originBodyFrame = Point3D{x, y, 0.0}, .yawRad = angleRad};
}

}  // namespace

LegMounting HexapodGeometry::getLegMounting(LegId legId) noexcept {
    return getLegMountingByIndex(static_cast<uint8_t>(legId));
}

LegMounting HexapodGeometry::getLegMountingByIndex(uint8_t legIndex) noexcept {
    switch (legIndex) {
        case 0:
            return createMountingFromAngle(kAngleFrontLeft);
        case 1:
            return createMountingFromAngle(kAngleMiddleLeft);
        case 2:
            return createMountingFromAngle(kAngleRearLeft);
        case 3:
            return createMountingFromAngle(kAngleRearRight);
        case 4:
            return createMountingFromAngle(kAngleMiddleRight);
        case 5:
            return createMountingFromAngle(kAngleFrontRight);
        default:
            return createMountingFromAngle(0.0);
    }
}

std::array<LegMounting, 6> HexapodGeometry::getAllLegMountings() noexcept {
    return {
        getLegMountingByIndex(0),
        getLegMountingByIndex(1),
        getLegMountingByIndex(2),
        getLegMountingByIndex(3),
        getLegMountingByIndex(4),
        getLegMountingByIndex(5)
    };
}

}  // namespace hexapod::core

