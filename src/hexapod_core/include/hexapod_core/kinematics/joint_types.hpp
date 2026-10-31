#ifndef HEXAPOD_CORE_KINEMATICS_JOINT_TYPES_HPP_
#define HEXAPOD_CORE_KINEMATICS_JOINT_TYPES_HPP_

#include <cstdint>
#include <numbers>

namespace hexapod::core {

struct JointLimits {
    double minAngleRad{-std::numbers::pi / 2.0};
    double maxAngleRad{std::numbers::pi / 2.0};
    double maxVelocityRadPerSec{5.0};
    double maxEffortNm{3.0};
};

struct JointCalibration {
    uint16_t zeroOffsetTicks{2048};  // 12-bit middle position (0..4095)
    int8_t direction{1};             // +1 (standard) or -1 (inverted)
    double ticksPerRadian{4096.0 / (2.0 * std::numbers::pi)};
};

struct LegJointAngles {
    double coxa{0.0};   // q1 (yaw, rotation around +Z)
    double femur{0.0};  // q2 (pitch, rotation around horizontal axis)
    double tibia{0.0};  // q3 (pitch, rotation collinear with femur)

    constexpr bool operator==(const LegJointAngles& other) const noexcept = default;
};

struct LegDimensions {
    double coxaLength{0.045};   // 45 mm (0.045 m)
    double femurLength{0.090};  // 90 mm (0.090 m)
    double tibiaLength{0.120};  // 120 mm (0.120 m)

    constexpr bool operator==(const LegDimensions& other) const noexcept = default;
};

}  // namespace hexapod::core

#endif  // HEXAPOD_CORE_KINEMATICS_JOINT_TYPES_HPP_

