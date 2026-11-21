#include "hexapod_core/locomotion/gait_scheduler.hpp"

#include <algorithm>
#include <cmath>

namespace hexapod::core {

GaitScheduler::GaitScheduler(const GaitConfig& config) noexcept
    : config_(config) {
    reset();
}

void GaitScheduler::setGait(GaitType type, double cyclePeriodSec) noexcept {
    switch (type) {
        case GaitType::kTripod:
            config_ = GaitConfig::createTripod(cyclePeriodSec);
            break;
        case GaitType::kQuadruped:
            config_ = GaitConfig::createQuadruped(cyclePeriodSec);
            break;
        case GaitType::kWave:
            config_ = GaitConfig::createWave(cyclePeriodSec);
            break;
        case GaitType::kCustom:
            config_.type = GaitType::kCustom;
            config_.cyclePeriodSec = cyclePeriodSec;
            break;
    }
    reset();
}

void GaitScheduler::setCustomGait(double dutyFactor, double cyclePeriodSec, const std::array<double, 6>& phaseOffsets) noexcept {
    config_.type = GaitType::kCustom;
    config_.dutyFactor = std::clamp(dutyFactor, 0.05, 0.95);
    config_.cyclePeriodSec = (cyclePeriodSec > 1e-4) ? cyclePeriodSec : 1e-4;
    config_.phaseOffsets = phaseOffsets;
    reset();
}

void GaitScheduler::reset() noexcept {
    masterPhase_ = 0.0;
    for (size_t i = 0; i < 6; ++i) {
        double phase = std::fmod(config_.phaseOffsets[i], 1.0);
        if (phase < 0.0) {
            phase += 1.0;
        }
        legPhases_[i] = phase;
        liftoffTriggered_[i] = false;
        touchdownTriggered_[i] = false;
    }
}

void GaitScheduler::update(double dt) noexcept {
    if (config_.cyclePeriodSec <= 1e-6) {
        return;
    }

    const double deltaPhase = dt / config_.cyclePeriodSec;
    masterPhase_ = std::fmod(masterPhase_ + deltaPhase, 1.0);
    if (masterPhase_ < 0.0) {
        masterPhase_ += 1.0;
    }

    for (size_t i = 0; i < 6; ++i) {
        const double oldPhase = legPhases_[i];
        double newPhase = std::fmod(masterPhase_ + config_.phaseOffsets[i], 1.0);
        if (newPhase < 0.0) {
            newPhase += 1.0;
        }
        legPhases_[i] = newPhase;

        // Liftoff occurs when transitioning from stance ([0, beta)) to swing ([beta, 1.0))
        liftoffTriggered_[i] = (oldPhase < config_.dutyFactor && newPhase >= config_.dutyFactor);

        // Touchdown occurs when wrapping around 1.0 from swing ([beta, 1.0)) to stance ([0, beta))
        touchdownTriggered_[i] = (oldPhase >= config_.dutyFactor && newPhase < config_.dutyFactor);
    }
}

double GaitScheduler::getLegPhase(size_t legIndex) const noexcept {
    if (legIndex >= 6) {
        return 0.0;
    }
    return legPhases_[legIndex];
}

LegPhaseState GaitScheduler::getLegState(size_t legIndex) const noexcept {
    return isStance(legIndex) ? LegPhaseState::kStance : LegPhaseState::kSwing;
}

bool GaitScheduler::isStance(size_t legIndex) const noexcept {
    if (legIndex >= 6) {
        return true;
    }
    return legPhases_[legIndex] < config_.dutyFactor;
}

bool GaitScheduler::isSwing(size_t legIndex) const noexcept {
    return !isStance(legIndex);
}

double GaitScheduler::getNormalizedStanceProgress(size_t legIndex) const noexcept {
    if (legIndex >= 6 || config_.dutyFactor <= 1e-6) {
        return 0.0;
    }
    const double phase = legPhases_[legIndex];
    if (phase < config_.dutyFactor) {
        return std::clamp(phase / config_.dutyFactor, 0.0, 1.0);
    }
    return 1.0;
}

double GaitScheduler::getNormalizedSwingProgress(size_t legIndex) const noexcept {
    if (legIndex >= 6) {
        return 0.0;
    }
    const double swingDurationPhase = 1.0 - config_.dutyFactor;
    if (swingDurationPhase <= 1e-6) {
        return 0.0;
    }
    const double phase = legPhases_[legIndex];
    if (phase >= config_.dutyFactor) {
        return std::clamp((phase - config_.dutyFactor) / swingDurationPhase, 0.0, 1.0);
    }
    return 0.0;
}

bool GaitScheduler::hasLiftoffOccurred(size_t legIndex) const noexcept {
    if (legIndex >= 6) {
        return false;
    }
    return liftoffTriggered_[legIndex];
}

bool GaitScheduler::hasTouchdownOccurred(size_t legIndex) const noexcept {
    if (legIndex >= 6) {
        return false;
    }
    return touchdownTriggered_[legIndex];
}

Point3D GaitScheduler::calculateStanceVelocity(const Point3D& linearVel,
                                              const Point3D& angularVel,
                                              const Point3D& footPositionBodyFrame) const noexcept {
    const Point3D rotationalVel = angularVel.cross(footPositionBodyFrame);
    return -(linearVel + rotationalVel);
}

Point3D GaitScheduler::calculateTouchdownTarget(const Point3D& nominalFootBodyFrame,
                                               const Point3D& linearVel,
                                               const Point3D& angularVel) const noexcept {
    const double stanceDuration = getStanceDuration();
    const Point3D rotationalVel = angularVel.cross(nominalFootBodyFrame);
    const Point3D bodyVelAtFoot = linearVel + rotationalVel;

    // Reach forward along the travel direction by half the stroke distance
    return nominalFootBodyFrame + bodyVelAtFoot * (0.5 * stanceDuration);
}

}  // namespace hexapod::core
