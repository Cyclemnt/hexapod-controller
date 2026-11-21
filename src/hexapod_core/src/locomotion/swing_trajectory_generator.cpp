#include "hexapod_core/locomotion/swing_trajectory_generator.hpp"

#include <algorithm>

namespace hexapod::core {

SwingTrajectoryGenerator::SwingTrajectoryGenerator(const SwingConfig& config) noexcept
    : config_(config) {}

void SwingTrajectoryGenerator::reset(const Point3D& liftoffPosition, const Point3D& targetPosition, double swingDurationSec) noexcept {
    liftoffPosition_ = liftoffPosition;
    targetPosition_ = targetPosition;
    swingDurationSec_ = (swingDurationSec > 1e-4) ? swingDurationSec : 1e-4;
    apexZ_ = std::max(liftoffPosition.z, targetPosition.z) + config_.stepHeight;

    const TrajectoryPoint startState{liftoffPosition, Point3D{0.0, 0.0, 0.0}, Point3D{0.0, 0.0, 0.0}};
    planTrajectory(startState, targetPosition, swingDurationSec_, apexZ_);
}

void SwingTrajectoryGenerator::replan(const TrajectoryPoint& currentState, const Point3D& newTarget,
                                     double remainingDurationSec, double stepHeightBoost) noexcept {
    liftoffPosition_ = currentState.position;
    targetPosition_ = newTarget;
    swingDurationSec_ = (remainingDurationSec > 1e-4) ? remainingDurationSec : 1e-4;
    apexZ_ = std::max(currentState.position.z, newTarget.z) + config_.stepHeight + stepHeightBoost;

    planTrajectory(currentState, newTarget, swingDurationSec_, apexZ_);
}

void SwingTrajectoryGenerator::planTrajectory(const TrajectoryPoint& startState, const Point3D& targetPos,
                                             double durationSec, double apexHeight) noexcept {
    // Horizontal quintic trajectories: zero velocity and acceleration at start (or matching startState) and end
    splineX_.setBoundary(startState.position.x, startState.velocity.x, startState.acceleration.x,
                         targetPos.x, 0.0, 0.0, durationSec);
    splineY_.setBoundary(startState.position.y, startState.velocity.y, startState.acceleration.y,
                         targetPos.y, 0.0, 0.0, durationSec);

    // Vertical trajectory split at apex for smooth clearance and descent
    const double ratio = std::clamp(config_.apexRatio, 0.1, 0.9);
    apexTimeSec_ = ratio * durationSec;
    const double descentDuration = durationSec - apexTimeSec_;

    // Ascent: reaches apex with zero vertical velocity and zero acceleration
    splineZAscent_.setBoundary(startState.position.z, startState.velocity.z, startState.acceleration.z,
                               apexHeight, 0.0, 0.0, apexTimeSec_);

    // Descent: reaches nominal ground with downward probing velocity and zero acceleration
    splineZDescent_.setBoundary(apexHeight, 0.0, 0.0,
                                targetPos.z, -config_.probeVelocity, 0.0, descentDuration);
}

TrajectoryPoint SwingTrajectoryGenerator::evaluate(double elapsedTimeSec) const noexcept {
    if (elapsedTimeSec < swingDurationSec_) {
        const double t = std::max(0.0, elapsedTimeSec);
        const double xPos = splineX_.evaluatePosition(t);
        const double xVel = splineX_.evaluateVelocity(t);
        const double xAcc = splineX_.evaluateAcceleration(t);

        const double yPos = splineY_.evaluatePosition(t);
        const double yVel = splineY_.evaluateVelocity(t);
        const double yAcc = splineY_.evaluateAcceleration(t);

        double zPos = 0.0;
        double zVel = 0.0;
        double zAcc = 0.0;

        if (t <= apexTimeSec_) {
            zPos = splineZAscent_.evaluatePosition(t);
            zVel = splineZAscent_.evaluateVelocity(t);
            zAcc = splineZAscent_.evaluateAcceleration(t);
        } else {
            const double descentTime = t - apexTimeSec_;
            zPos = splineZDescent_.evaluatePosition(descentTime);
            zVel = splineZDescent_.evaluateVelocity(descentTime);
            zAcc = splineZDescent_.evaluateAcceleration(descentTime);
        }

        return TrajectoryPoint{
            Point3D{xPos, yPos, zPos},
            Point3D{xVel, yVel, zVel},
            Point3D{xAcc, yAcc, zAcc}
        };
    }

    // elapsedTimeSec >= swingDurationSec_: Probing continuation mode below nominal target
    const double dtProbe = elapsedTimeSec - swingDurationSec_;
    const double maxDrop = config_.maxProbingDistance;
    const double desiredDrop = dtProbe * config_.probeVelocity;
    const double actualDrop = std::min(desiredDrop, maxDrop);

    const double zPos = targetPosition_.z - actualDrop;
    const double zVel = (actualDrop < maxDrop) ? -config_.probeVelocity : 0.0;
    const double zAcc = 0.0;

    return TrajectoryPoint{
        Point3D{targetPosition_.x, targetPosition_.y, zPos},
        Point3D{0.0, 0.0, zVel},
        Point3D{0.0, 0.0, zAcc}
    };
}

}  // namespace hexapod::core
