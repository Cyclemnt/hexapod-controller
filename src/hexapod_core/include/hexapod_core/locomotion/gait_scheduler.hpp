#ifndef HEXAPOD_CORE_LOCOMOTION_GAIT_SCHEDULER_HPP_
#define HEXAPOD_CORE_LOCOMOTION_GAIT_SCHEDULER_HPP_

#include <array>
#include <cstddef>
#include <cstdint>

#include "hexapod_core/math/point_3d.hpp"

namespace hexapod::core {

enum class GaitType : uint8_t {
    kTripod = 0,
    kQuadruped = 1,
    kWave = 2,
    kCustom = 3
};

enum class LegPhaseState : uint8_t {
    kStance = 0,
    kSwing = 1
};

struct GaitConfig {
    GaitType type{GaitType::kTripod};
    double dutyFactor{0.50};          // Stance duty factor beta in (0.0, 1.0)
    double cyclePeriodSec{1.0};       // Total cycle period T in seconds
    std::array<double, 6> phaseOffsets{0.0, 0.5, 0.0, 0.5, 0.0, 0.5};

    static constexpr GaitConfig createTripod(double cyclePeriodSec = 1.0) noexcept {
        return GaitConfig{
            .type = GaitType::kTripod,
            .dutyFactor = 0.50,
            .cyclePeriodSec = cyclePeriodSec,
            .phaseOffsets = {0.0, 0.5, 0.0, 0.5, 0.0, 0.5}
        };
    }

    static constexpr GaitConfig createQuadruped(double cyclePeriodSec = 1.5) noexcept {
        return GaitConfig{
            .type = GaitType::kQuadruped,
            .dutyFactor = 2.0 / 3.0,
            .cyclePeriodSec = cyclePeriodSec,
            .phaseOffsets = {0.0, 2.0 / 3.0, 1.0 / 3.0, 0.0, 2.0 / 3.0, 1.0 / 3.0}
        };
    }

    static constexpr GaitConfig createWave(double cyclePeriodSec = 3.0) noexcept {
        return GaitConfig{
            .type = GaitType::kWave,
            .dutyFactor = 5.0 / 6.0,
            .cyclePeriodSec = cyclePeriodSec,
            .phaseOffsets = {2.0 / 6.0, 1.0 / 6.0, 0.0, 3.0 / 6.0, 4.0 / 6.0, 5.0 / 6.0}
        };
    }
};

// Manages the continuous phase progression of all 6 legs,
// coordinates stance/swing transitions, and generates stance velocities
// and touchdown target foot placements for omnidirectional translation and rotation.
class GaitScheduler {
public:
    explicit GaitScheduler(const GaitConfig& config = GaitConfig::createTripod(1.0)) noexcept;

    void setGait(GaitType type, double cyclePeriodSec = 1.0) noexcept;
    void setCustomGait(double dutyFactor, double cyclePeriodSec, const std::array<double, 6>& phaseOffsets) noexcept;

    void reset() noexcept;
    void update(double dt) noexcept;

    // Continuous master phase in [0.0, 1.0)
    [[nodiscard]] double getMasterPhase() const noexcept { return masterPhase_; }

    // Continuous leg phase phi_i in [0.0, 1.0)
    [[nodiscard]] double getLegPhase(size_t legIndex) const noexcept;

    // Stance or Swing state of leg i
    [[nodiscard]] LegPhaseState getLegState(size_t legIndex) const noexcept;
    [[nodiscard]] bool isStance(size_t legIndex) const noexcept;
    [[nodiscard]] bool isSwing(size_t legIndex) const noexcept;

    // Normalized progress in [0.0, 1.0) within stance or swing sub-cycle
    [[nodiscard]] double getNormalizedStanceProgress(size_t legIndex) const noexcept;
    [[nodiscard]] double getNormalizedSwingProgress(size_t legIndex) const noexcept;

    // Event flags: true only for the single update step in which the transition occurred
    [[nodiscard]] bool hasLiftoffOccurred(size_t legIndex) const noexcept;
    [[nodiscard]] bool hasTouchdownOccurred(size_t legIndex) const noexcept;

    // Stance foot velocity: v_foot = -(v_lin + omega x r)
    [[nodiscard]] Point3D calculateStanceVelocity(const Point3D& linearVel, const Point3D& angularVel,
                                                  const Point3D& footPositionBodyFrame) const noexcept;

    // Nominal touchdown target placement ahead of neutral foot position
    [[nodiscard]] Point3D calculateTouchdownTarget(const Point3D& nominalFootBodyFrame,
                                                  const Point3D& linearVel,
                                                  const Point3D& angularVel) const noexcept;

    [[nodiscard]] const GaitConfig& getConfig() const noexcept { return config_; }
    [[nodiscard]] double getStanceDuration() const noexcept { return config_.dutyFactor * config_.cyclePeriodSec; }
    [[nodiscard]] double getSwingDuration() const noexcept { return (1.0 - config_.dutyFactor) * config_.cyclePeriodSec; }
    [[nodiscard]] double getCyclePeriod() const noexcept { return config_.cyclePeriodSec; }
    [[nodiscard]] double getDutyFactor() const noexcept { return config_.dutyFactor; }

private:
    GaitConfig config_{};
    double masterPhase_{0.0};
    std::array<double, 6> legPhases_{};
    std::array<bool, 6> liftoffTriggered_{};
    std::array<bool, 6> touchdownTriggered_{};
};

}  // namespace hexapod::core

#endif  // HEXAPOD_CORE_LOCOMOTION_GAIT_SCHEDULER_HPP_
