#include "hexapod_core/kinematics/analytical_leg_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace hexapod::core {

AnalyticalLegKinematics::AnalyticalLegKinematics(const LegDimensions& dimensions) noexcept
    : dimensions_(dimensions),
      effectiveTibiaLength_(std::hypot(dimensions.tibiaFootX, dimensions.tibiaFootZ)),
      tibiaAngleOffset_(std::atan2(dimensions.tibiaFootX, dimensions.tibiaFootZ)) {}

Point3D AnalyticalLegKinematics::forwardKinematics(const LegJointAngles& jointAngles) const noexcept {
    const double coxaAngle = jointAngles.coxa;
    const double femurAngle = jointAngles.femur;
    const double tibiaAngle = jointAngles.tibia;

    // Convention: at q2=0, femur is horizontal (+X). At q3=0, tibia drops along -Z with radial offset +tibiaFootX.
    const double sumAngle = femurAngle + tibiaAngle;
    const double cosSum = std::cos(sumAngle);
    const double sinSum = std::sin(sumAngle);

    const double r = dimensions_.coxaLength +
                     dimensions_.femurLength * std::cos(femurAngle) +
                     dimensions_.tibiaFootX * cosSum +
                     dimensions_.tibiaFootZ * sinSum;

    const double x = r * std::cos(coxaAngle);
    const double y = r * std::sin(coxaAngle);
    const double z = dimensions_.femurLength * std::sin(femurAngle) +
                     dimensions_.tibiaFootX * sinSum -
                     dimensions_.tibiaFootZ * cosSum;

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
    const double l3 = effectiveTibiaLength_;

    // Workspace boundary check
    const double minReach = std::abs(l2 - l3);
    const double maxReach = l2 + l3;
    if (distance > maxReach || distance < minReach) {
        return std::nullopt;
    }

    // 3. Angle gamma between femur and effective knee-to-foot segment: d^2 = l2^2 + l3^2 - 2*l2*l3*cos(gamma)
    const double cosGamma = (l2 * l2 + l3 * l3 - distanceSquared) / (2.0 * l2 * l3);
    const double clampedCosGamma = std::clamp(cosGamma, -1.0, 1.0);
    const double gamma = std::acos(clampedCosGamma);

    // At nominal standing pose (q3 = 0), gamma = pi/2 + atan2(tibiaFootX, tibiaFootZ).
    const double tibiaAngle = gamma - (std::numbers::pi / 2.0 + tibiaAngleOffset_);

    // 4. Femur joint (pitch)
    const double alpha = std::atan2(z, radialDistance);
    const double beta = std::atan2(l3 * std::sin(gamma), l2 - l3 * clampedCosGamma);
    const double femurAngle = alpha + beta;

    return LegJointAngles{.coxa = coxaAngle, .femur = femurAngle, .tibia = tibiaAngle};
}

bool AnalyticalLegKinematics::isReachable(const Point3D& targetInLegFrame) const noexcept {
    const double radialDistance = std::hypot(targetInLegFrame.x, targetInLegFrame.y) - dimensions_.coxaLength;
    const double distance = std::hypot(radialDistance, targetInLegFrame.z);
    const double minReach = std::abs(dimensions_.femurLength - effectiveTibiaLength_);
    const double maxReach = dimensions_.femurLength + effectiveTibiaLength_;

    return distance >= minReach && distance <= maxReach;
}

}  // namespace hexapod::core
