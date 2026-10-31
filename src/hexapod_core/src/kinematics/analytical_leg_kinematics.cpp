#include "hexapod_core/kinematics/analytical_leg_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace hexapod::core {

AnalyticalLegKinematics::AnalyticalLegKinematics(const LegDimensions& dimensions) noexcept : dimensions_(dimensions) {}

Point3D AnalyticalLegKinematics::forwardKinematics(const LegJointAngles& jointAngles) const noexcept {
    const double coxaAngle = jointAngles.coxa;
    const double femurAngle = jointAngles.femur;
    const double tibiaAngle = jointAngles.tibia;

    // Radial distance from coxa joint in the sagittal plane
    const double r = dimensions_.coxaLength +
                     dimensions_.femurLength * std::cos(femurAngle) +
                     dimensions_.tibiaLength * std::cos(femurAngle + tibiaAngle);

    const double x = r * std::cos(coxaAngle);
    const double y = r * std::sin(coxaAngle);
    const double z = dimensions_.femurLength * std::sin(femurAngle) +
                     dimensions_.tibiaLength * std::sin(femurAngle + tibiaAngle);

    return Point3D{x, y, z};
}

std::optional<LegJointAngles> AnalyticalLegKinematics::inverseKinematics(const Point3D& targetInLegFrame) const noexcept {
    const double x = targetInLegFrame.x;
    const double y = targetInLegFrame.y;
    const double z = targetInLegFrame.z;

    // 1. Coxa joint (yaw)
    const double coxaAngle = std::atan2(y, x);

    // 2. Project target onto sagittal plane
    const double radialDistance = std::hypot(x, y) - dimensions_.coxaLength;
    const double distanceSquared = radialDistance * radialDistance + z * z;
    const double distance = std::sqrt(distanceSquared);

    const double l2 = dimensions_.femurLength;
    const double l3 = dimensions_.tibiaLength;

    // Workspace boundary check
    const double minReach = std::abs(l2 - l3);
    const double maxReach = l2 + l3;
    if (distance > maxReach || distance < minReach) {
        return std::nullopt;
    }

    // 3. Tibia joint (pitch) using Law of Cosines
    // d^2 = l2^2 + l3^2 + 2 * l2 * l3 * cos(tibiaAngle)
    const double cosTibia = (distanceSquared - l2 * l2 - l3 * l3) / (2.0 * l2 * l3);
    const double clampedCosTibia = std::clamp(cosTibia, -1.0, 1.0);

    // Knee bent downwards: tibiaAngle <= 0
    const double sinTibia = -std::sqrt(std::max(0.0, 1.0 - clampedCosTibia * clampedCosTibia));
    const double tibiaAngle = std::atan2(sinTibia, clampedCosTibia);

    // 4. Femur joint (pitch)
    // q2 = atan2(z, r) - atan2(l3 * sin(q3), l2 + l3 * cos(q3))
    const double angleFemurOffset = std::atan2(l3 * sinTibia, l2 + l3 * clampedCosTibia);
    const double femurAngle = std::atan2(z, radialDistance) - angleFemurOffset;

    return LegJointAngles{.coxa = coxaAngle, .femur = femurAngle, .tibia = tibiaAngle};
}

bool AnalyticalLegKinematics::isReachable(const Point3D& targetInLegFrame) const noexcept {
    const double radialDistance = std::hypot(targetInLegFrame.x, targetInLegFrame.y) - dimensions_.coxaLength;
    const double distance = std::hypot(radialDistance, targetInLegFrame.z);
    const double minReach = std::abs(dimensions_.femurLength - dimensions_.tibiaLength);
    const double maxReach = dimensions_.femurLength + dimensions_.tibiaLength;

    return distance >= minReach && distance <= maxReach;
}

}  // namespace hexapod::core

