#include "hexapod_core/actuator/joint.hpp"

#include <algorithm>
#include <cmath>

namespace hexapod::core {

Joint::Joint(uint8_t id, const JointLimits& limits, const JointCalibration& calibration) noexcept
    : id_(id), limits_(limits), calibration_(calibration) {}

void Joint::setTargetAngle(double angleRad) {
    targetAngleRad_ = std::clamp(angleRad, limits_.minAngleRad, limits_.maxAngleRad);
}

void Joint::setPresentState(double angleRad, double velocityRadPerSec, double effortLoad) {
    presentAngleRad_ = angleRad;
    presentVelocityRadPerSec_ = velocityRadPerSec;
    presentEffortLoad_ = effortLoad;
}

uint16_t Joint::radiansToTicks(double angleRad) const noexcept {
    const double clampedAngle = std::clamp(angleRad, limits_.minAngleRad, limits_.maxAngleRad);
    const auto deltaTicks = static_cast<int32_t>(std::round(calibration_.direction * clampedAngle * calibration_.ticksPerRadian));
    const int32_t rawTicks = static_cast<int32_t>(calibration_.zeroOffsetTicks) + deltaTicks;
    return static_cast<uint16_t>(std::clamp(rawTicks, 0, 4095));
}

double Joint::ticksToRadians(uint16_t ticks) const noexcept {
    const int32_t deltaTicks = static_cast<int32_t>(ticks) - static_cast<int32_t>(calibration_.zeroOffsetTicks);
    return (static_cast<double>(deltaTicks) / calibration_.ticksPerRadian) * calibration_.direction;
}

}  // namespace hexapod::core

