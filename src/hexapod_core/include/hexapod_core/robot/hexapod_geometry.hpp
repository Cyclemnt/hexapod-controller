#ifndef HEXAPOD_CORE_ROBOT_HEXAPOD_GEOMETRY_HPP_
#define HEXAPOD_CORE_ROBOT_HEXAPOD_GEOMETRY_HPP_

#include <array>
#include <cstdint>
#include <numbers>

#include "hexapod_core/kinematics/joint_types.hpp"
#include "hexapod_core/robot/leg_mounting.hpp"

namespace hexapod::core {

enum class LegId : uint8_t {
    kFrontLeft = 0,
    kMiddleLeft = 1,
    kRearLeft = 2,
    kRearRight = 3,
    kMiddleRight = 4,
    kFrontRight = 5
};

class HexapodGeometry {
public:
    // Chassis and link physical dimensions in meters
    static constexpr double kChassisRadius{0.130};  // Distance center to corner (130 mm)
    static constexpr double kCoxaLength{0.045};     // 45 mm
    static constexpr double kFemurLength{0.090};    // 90 mm
    static constexpr double kTibiaLength{0.120};    // 120 mm

    // Angles of leg mounting vertices relative to body frame center (radians)
    // Front of the robot (camera) is at 0 degrees, midpoint of the front edge.
    static constexpr double kAngleFrontLeft{std::numbers::pi / 6.0};         // +30 deg
    static constexpr double kAngleMiddleLeft{std::numbers::pi / 2.0};        // +90 deg
    static constexpr double kAngleRearLeft{5.0 * std::numbers::pi / 6.0};    // +150 deg
    static constexpr double kAngleRearRight{-5.0 * std::numbers::pi / 6.0};  // -150 deg
    static constexpr double kAngleMiddleRight{-std::numbers::pi / 2.0};      // -90 deg
    static constexpr double kAngleFrontRight{-std::numbers::pi / 6.0};       // -30 deg

    [[nodiscard]] static constexpr LegDimensions getDefaultLegDimensions() noexcept {
        return LegDimensions{.coxaLength = kCoxaLength, .femurLength = kFemurLength, .tibiaLength = kTibiaLength};
    }

    [[nodiscard]] static LegMounting getLegMounting(LegId legId) noexcept;
    [[nodiscard]] static LegMounting getLegMountingByIndex(uint8_t legIndex) noexcept;
    [[nodiscard]] static std::array<LegMounting, 6> getAllLegMountings() noexcept;
};

}  // namespace hexapod::core

#endif  // HEXAPOD_CORE_ROBOT_HEXAPOD_GEOMETRY_HPP_

