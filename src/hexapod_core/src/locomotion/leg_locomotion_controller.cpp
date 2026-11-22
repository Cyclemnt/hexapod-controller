#include "hexapod_core/locomotion/leg_locomotion_controller.hpp"

#include <algorithm>

namespace hexapod::core {

LegLocomotionController::LegLocomotionController(uint8_t legId, std::shared_ptr<ILeg> leg,
                                                 const LegLocomotionConfig& config) noexcept
    : legId_(legId),
      leg_(std::move(leg)),
      config_(config),
      trajectoryGen_(config.swingConfig),
      contactDetector_(config.contactConfig) {
    if (leg_) {
        stancePositionBodyFrame_ = leg_->getCurrentFootInBodyFrame();
        targetFootBodyFrame_ = stancePositionBodyFrame_;
    }
}

void LegLocomotionController::setConfig(const LegLocomotionConfig& config) noexcept {
    config_ = config;
    trajectoryGen_.setConfig(config.swingConfig);
    contactDetector_.setConfig(config.contactConfig);
}

Point3D LegLocomotionController::getCurrentFootBodyFrame() const noexcept {
    if (!leg_) {
        return stancePositionBodyFrame_;
    }
    return leg_->getCurrentFootInBodyFrame();
}

void LegLocomotionController::forceStance(const Point3D& stancePositionBodyFrame) noexcept {
    state_ = LegState::kStance;
    swingElapsedTime_ = 0.0;
    stancePositionBodyFrame_ = stancePositionBodyFrame;
    targetFootBodyFrame_ = stancePositionBodyFrame;
    earlyTouchdownTriggered_ = false;
    obstacleReflexTriggered_ = false;
    if (leg_) {
        leg_->setTargetInBodyFrame(stancePositionBodyFrame_);
    }
}

void LegLocomotionController::initiateSwing(const Point3D& startPositionNominal,
                                            const Point3D& plannedTouchdownTarget,
                                            double swingDurationSec) noexcept {
    state_ = LegState::kSwing;
    swingElapsedTime_ = 0.0;
    earlyTouchdownTriggered_ = false;
    obstacleReflexTriggered_ = false;

    trajectoryGen_.reset(startPositionNominal, plannedTouchdownTarget, swingDurationSec);
    targetFootBodyFrame_ = startPositionNominal;
}

void LegLocomotionController::update(double dt, const Point3D& measuredFootNominal,
                                     const Point3D& stanceVelocity,
                                     const Point3D& plannedTouchdownTarget,
                                     double swingDurationSec) noexcept {
    earlyTouchdownTriggered_ = false;
    obstacleReflexTriggered_ = false;

    if (!leg_) {
        return;
    }

    switch (state_) {
        case LegState::kStance: {
            stancePositionBodyFrame_ += stanceVelocity * dt;
            targetFootBodyFrame_ = stancePositionBodyFrame_;
            break;
        }

        case LegState::kSwing: {
            swingElapsedTime_ += dt;

            // 1. Check for obstacle collision during ascending / advancing phase
            if (contactDetector_.detectObstacle(*leg_, swingElapsedTime_, trajectoryGen_.getApexTime())) {
                state_ = LegState::kReflex;
                obstacleReflexTriggered_ = true;

                const TrajectoryPoint curState = trajectoryGen_.evaluate(swingElapsedTime_);
                const double remaining = std::max(0.10, swingDurationSec - swingElapsedTime_);
                trajectoryGen_.replan(curState, plannedTouchdownTarget, remaining, config_.stepHeightBoost);
                swingElapsedTime_ = 0.0;

                const TrajectoryPoint replannedPoint = trajectoryGen_.evaluate(0.0);
                targetFootBodyFrame_ = replannedPoint.position;
                break;
            }

            // 2. Check for ground contact only during descending phase
            if (swingElapsedTime_ >= trajectoryGen_.getApexTime() &&
                contactDetector_.detectTouchdown(*leg_, swingElapsedTime_)) {
                state_ = LegState::kStance;
                earlyTouchdownTriggered_ = true;
                stancePositionBodyFrame_ = measuredFootNominal;
                targetFootBodyFrame_ = stancePositionBodyFrame_;
                break;
            }

            // 3. Check for completion of nominal swing without ground contact
            if (swingElapsedTime_ >= trajectoryGen_.getSwingDuration()) {
                state_ = LegState::kProbing;
            }

            const TrajectoryPoint pt = trajectoryGen_.evaluate(swingElapsedTime_);
            targetFootBodyFrame_ = pt.position;
            break;
        }

        case LegState::kReflex: {
            swingElapsedTime_ += dt;

            // Check for ground contact during reflex descent
            if (swingElapsedTime_ >= trajectoryGen_.getApexTime() &&
                contactDetector_.detectTouchdown(*leg_, swingElapsedTime_)) {
                state_ = LegState::kStance;
                earlyTouchdownTriggered_ = true;
                stancePositionBodyFrame_ = measuredFootNominal;
                targetFootBodyFrame_ = stancePositionBodyFrame_;
                break;
            }

            if (swingElapsedTime_ >= trajectoryGen_.getSwingDuration()) {
                state_ = LegState::kProbing;
            }

            const TrajectoryPoint pt = trajectoryGen_.evaluate(swingElapsedTime_);
            targetFootBodyFrame_ = pt.position;
            break;
        }

        case LegState::kProbing: {
            swingElapsedTime_ += dt;

            // Touchdown detected at bottom of hole
            if (contactDetector_.detectTouchdown(*leg_, swingElapsedTime_)) {
                state_ = LegState::kStance;
                earlyTouchdownTriggered_ = true;
                stancePositionBodyFrame_ = measuredFootNominal;
                targetFootBodyFrame_ = stancePositionBodyFrame_;
                break;
            }

            // Safety limit on probing depth
            const double maxProbeTime = trajectoryGen_.getSwingDuration() +
                config_.swingConfig.maxProbingDistance / config_.swingConfig.probeVelocity;
            if (swingElapsedTime_ >= maxProbeTime) {
                state_ = LegState::kStance;
                stancePositionBodyFrame_ = measuredFootNominal;
                targetFootBodyFrame_ = stancePositionBodyFrame_;
                break;
            }

            const TrajectoryPoint pt = trajectoryGen_.evaluate(swingElapsedTime_);
            targetFootBodyFrame_ = pt.position;
            break;
        }
    }
}

}  // namespace hexapod::core
