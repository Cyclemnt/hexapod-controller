#ifndef HEXAPOD_CORE_LOCOMOTION_LEG_LOCOMOTION_CONTROLLER_HPP_
#define HEXAPOD_CORE_LOCOMOTION_LEG_LOCOMOTION_CONTROLLER_HPP_

#include <memory>

#include "hexapod_core/locomotion/contact_detector.hpp"
#include "hexapod_core/locomotion/swing_trajectory_generator.hpp"
#include "hexapod_core/math/point_3d.hpp"
#include "hexapod_core/robot/i_leg.hpp"

namespace hexapod::core {

enum class LegState : uint8_t {
    kStance = 0,
    kSwing = 1,
    kProbing = 2,
    kReflex = 3
};

struct LegLocomotionConfig {
    SwingConfig swingConfig{};
    ContactDetectorConfig contactConfig{};
    double stepHeightBoost{0.025};  // 25 mm extra clearance on obstacle reflex
};

// Manages individual leg locomotion state machine: Stance, Swing, Probing, and Reflex
class LegLocomotionController {
public:
    LegLocomotionController(uint8_t legId, std::shared_ptr<ILeg> leg,
                            const LegLocomotionConfig& config = LegLocomotionConfig{}) noexcept;

    // Updates leg state and executes motion for time step dt
    void update(double dt, const Point3D& measuredFootNominal, const Point3D& stanceVelocity,
                const Point3D& plannedTouchdownTarget, double swingDurationSec) noexcept;

    // Forces leg into stance state at a specific position
    void forceStance(const Point3D& stancePositionBodyFrame) noexcept;

    // Commands liftoff transition into swing from nominal start position
    void initiateSwing(const Point3D& startPositionNominal, const Point3D& plannedTouchdownTarget,
                       double swingDurationSec) noexcept;

    [[nodiscard]] uint8_t getLegId() const noexcept { return legId_; }
    [[nodiscard]] LegState getState() const noexcept { return state_; }
    [[nodiscard]] bool isStance() const noexcept { return state_ == LegState::kStance; }
    [[nodiscard]] bool isSwing() const noexcept { return state_ == LegState::kSwing; }
    [[nodiscard]] bool isProbing() const noexcept { return state_ == LegState::kProbing; }
    [[nodiscard]] bool isReflex() const noexcept { return state_ == LegState::kReflex; }

    [[nodiscard]] double getSwingElapsedTime() const noexcept { return swingElapsedTime_; }
    [[nodiscard]] Point3D getStancePositionBodyFrame() const noexcept { return stancePositionBodyFrame_; }
    [[nodiscard]] Point3D getCurrentFootBodyFrame() const noexcept;
    [[nodiscard]] Point3D getTargetFootBodyFrame() const noexcept { return targetFootBodyFrame_; }

    [[nodiscard]] const LegLocomotionConfig& getConfig() const noexcept { return config_; }
    void setConfig(const LegLocomotionConfig& config) noexcept;

    [[nodiscard]] const SwingTrajectoryGenerator& getTrajectoryGenerator() const noexcept { return trajectoryGen_; }
    [[nodiscard]] const ContactDetector& getContactDetector() const noexcept { return contactDetector_; }

    [[nodiscard]] bool hasEarlyTouchdownOccurred() const noexcept { return earlyTouchdownTriggered_; }
    [[nodiscard]] bool hasObstacleReflexOccurred() const noexcept { return obstacleReflexTriggered_; }

private:
    uint8_t legId_{0};
    std::shared_ptr<ILeg> leg_{};
    LegLocomotionConfig config_{};

    LegState state_{LegState::kStance};
    SwingTrajectoryGenerator trajectoryGen_{};
    ContactDetector contactDetector_{};

    double swingElapsedTime_{0.0};
    Point3D stancePositionBodyFrame_{};
    Point3D targetFootBodyFrame_{};

    bool earlyTouchdownTriggered_{false};
    bool obstacleReflexTriggered_{false};
};

}  // namespace hexapod::core

#endif  // HEXAPOD_CORE_LOCOMOTION_LEG_LOCOMOTION_CONTROLLER_HPP_
