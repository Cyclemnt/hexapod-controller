#include "hexapod_core/locomotion/contact_detector.hpp"

#include <cmath>

namespace hexapod::core {

ContactDetector::ContactDetector(const ContactDetectorConfig& config) noexcept
    : config_(config) {}

bool ContactDetector::detectTouchdown(const ILeg& leg, double swingElapsedTime) const noexcept {
    // Blanking window: avoid false positives during early liftoff acceleration
    if (swingElapsedTime < config_.minSwingDurationSec) {
        return false;
    }

    const auto femur = leg.getFemurJoint();
    const auto tibia = leg.getTibiaJoint();
    if (!femur || !tibia) {
        return false;
    }

    const double femurEffort = std::abs(femur->getPresentEffort());
    const double tibiaEffort = std::abs(tibia->getPresentEffort());
    const double maxEffort = std::max(femurEffort, tibiaEffort);

    // Critical impact effort alone is sufficient
    if (maxEffort >= config_.criticalEffortThreshold) {
        return true;
    }

    const double femurErr = std::abs(femur->getTargetAngle() - femur->getPresentAngle());
    const double tibiaErr = std::abs(tibia->getTargetAngle() - tibia->getPresentAngle());
    const double maxErr = std::max(femurErr, tibiaErr);

    // Require both moderate effort and significant tracking error to reject pure inertial deceleration
    return (maxEffort >= config_.effortThreshold && maxErr >= config_.trackingErrorThresholdRad);
}

bool ContactDetector::detectObstacle(const ILeg& leg, double swingElapsedTime, double apexTimeSec) const noexcept {
    // Obstacle reflex is monitored during ascending / advancing swing
    if (swingElapsedTime >= apexTimeSec) {
        return false;
    }

    const auto coxa = leg.getCoxaJoint();
    const auto femur = leg.getFemurJoint();
    if (!coxa || !femur) {
        return false;
    }

    const double coxaEffort = std::abs(coxa->getPresentEffort());
    const double femurEffort = std::abs(femur->getPresentEffort());
    if (coxaEffort >= config_.obstacleEffortThreshold || femurEffort >= config_.obstacleEffortThreshold) {
        return true;
    }

    const double coxaErr = std::abs(coxa->getTargetAngle() - coxa->getPresentAngle());
    return (coxaEffort >= config_.effortThreshold && coxaErr >= config_.trackingErrorThresholdRad);
}

}  // namespace hexapod::core
