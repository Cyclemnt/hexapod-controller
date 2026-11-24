#include "hexapod_core/locomotion/locomotion_engine.hpp"

#include <array>
#include <cmath>
#include <numbers>

#include "hexapod_core/actuator/joint.hpp"
#include "hexapod_core/kinematics/analytical_leg_kinematics.hpp"
#include "hexapod_core/robot/hexapod_geometry.hpp"
#include "hexapod_core/robot/leg.hpp"

namespace hexapod::core {

LocomotionEngine::LocomotionEngine(std::array<std::shared_ptr<ILeg>, 6> legs,
                                   const LocomotionEngineConfig& config) noexcept
    : config_(config),
      legs_(std::move(legs)),
      gaitScheduler_(config.gaitConfig),
      terrainEstimator_(config.terrainConfig) {
    legControllers_.reserve(6);
    for (size_t i = 0; i < 6; ++i) {
        legControllers_.emplace_back(static_cast<uint8_t>(i), legs_[i], config.legConfig);
    }
    initNominalFeet();
    requestStanding();
}

std::unique_ptr<LocomotionEngine> LocomotionEngine::createDefault(const LocomotionEngineConfig& config) {
    const auto dims = HexapodGeometry::getDefaultLegDimensions();
    const JointLimits limits{};
    const JointCalibration calib{};

    std::array<std::shared_ptr<ILeg>, 6> legs{};
    for (size_t i = 0; i < 6; ++i) {
        const auto mounting = HexapodGeometry::getLegMountingByIndex(static_cast<uint8_t>(i));
        auto kinematics = std::make_unique<AnalyticalLegKinematics>(dims);

        auto coxaJoint = std::make_shared<Joint>(static_cast<uint8_t>(i * 3 + 1), limits, calib);
        auto femurJoint = std::make_shared<Joint>(static_cast<uint8_t>(i * 3 + 2), limits, calib);
        auto tibiaJoint = std::make_shared<Joint>(static_cast<uint8_t>(i * 3 + 3), limits, calib);

        legs[i] = std::make_shared<Leg>(static_cast<uint8_t>(i), mounting, std::move(kinematics),
                                        coxaJoint, femurJoint, tibiaJoint);
    }

    return std::make_unique<LocomotionEngine>(std::move(legs), config);
}

void LocomotionEngine::initNominalFeet() noexcept {
    for (size_t i = 0; i < 6; ++i) {
        if (!legs_[i]) {
            continue;
        }
        const Point3D nominalLegFrame{config_.stanceRadius, 0.0, config_.targetStandingHeight};
        nominalFeetInBody_[i] = legs_[i]->getMounting().getTransform().transformPoint(nominalLegFrame);
        legControllers_[i].forceStance(nominalFeetInBody_[i]);
    }
}

Point3D LocomotionEngine::getNominalFootInBody(size_t legIndex) const noexcept {
    if (legIndex >= 6) {
        return Point3D{};
    }
    return nominalFeetInBody_[legIndex];
}

const LegLocomotionController& LocomotionEngine::getLegController(size_t legIndex) const noexcept {
    return legControllers_.at(legIndex);
}

LegLocomotionController& LocomotionEngine::getLegController(size_t legIndex) noexcept {
    return legControllers_.at(legIndex);
}

void LocomotionEngine::setConfig(const LocomotionEngineConfig& config) noexcept {
    config_ = config;
    if (config.gaitConfig.type == GaitType::kCustom) {
        gaitScheduler_.setCustomGait(config.gaitConfig.dutyFactor, config.gaitConfig.cyclePeriodSec,
                                     config.gaitConfig.phaseOffsets);
    } else {
        gaitScheduler_.setGait(config.gaitConfig.type, config.gaitConfig.cyclePeriodSec);
    }
    terrainEstimator_.setConfig(config.terrainConfig);
    for (auto& lc : legControllers_) {
        lc.setConfig(config.legConfig);
    }
    initNominalFeet();
}

void LocomotionEngine::requestStanding() noexcept {
    if (state_ == LocomotionState::kStanding) {
        return;
    }
    state_ = LocomotionState::kStanding;
    transitionProgress_ = 0.0;
    isTransitioning_ = true;
    transitionToStanding_ = true;
}

void LocomotionEngine::requestSitting() noexcept {
    // Ignore request if currently locomoting; the robot must first return to standing
    if (state_ == LocomotionState::kSitting || state_ == LocomotionState::kLocomotion) {
        return;
    }
    state_ = LocomotionState::kSitting;
    transitionProgress_ = 0.0;
    isTransitioning_ = true;
    transitionToStanding_ = false;
    previousEffPose_ = BodyPoseCommand{};
}

void LocomotionEngine::setVelocityCommand(const Point3D& linearVel, const Point3D& angularVel) noexcept {
    cmdLinearVel_ = linearVel;
    cmdAngularVel_ = angularVel;
}

void LocomotionEngine::setBodyPoseCommand(const BodyPoseCommand& pose) noexcept {
    userBodyPose_ = pose;
}

void LocomotionEngine::setImuOrientation(double rollRad, double pitchRad, double yawRad) noexcept {
    imuRoll_ = rollRad;
    imuPitch_ = pitchRad;
    imuYaw_ = yawRad;
}

void LocomotionEngine::update(double dt) noexcept {
    switch (state_) {
        case LocomotionState::kSitting:
            updateSitting(dt);
            break;
        case LocomotionState::kStanding:
            updateStanding(dt);
            break;
        case LocomotionState::kLocomotion:
            updateLocomotion(dt);
            break;
    }
}

void LocomotionEngine::updateSitting(double dt) noexcept {
    if (isTransitioning_ && !transitionToStanding_) {
        // Smoothly lower down from standing to sitting
        transitionProgress_ += dt / std::max(1e-4, config_.standingTransitionTimeSec);
        if (transitionProgress_ >= 1.0) {
            transitionProgress_ = 1.0;
            isTransitioning_ = false;
        }

        const double s = 0.5 * (1.0 - std::cos(std::numbers::pi * transitionProgress_));

        for (size_t i = 0; i < 6; ++i) {
            Point3D footPos = nominalFeetInBody_[i];
            const double nominalZ = nominalFeetInBody_[i].z;
            const double sitZ = nominalZ + (config_.sittingHeight - config_.targetStandingHeight);
            footPos.z = nominalZ + s * (sitZ - nominalZ);
            legControllers_[i].forceStance(footPos);
        }
        return;
    }

    // Hold sitting posture
    for (size_t i = 0; i < 6; ++i) {
        Point3D footPos = nominalFeetInBody_[i];
        const double nominalZ = nominalFeetInBody_[i].z;
        footPos.z = nominalZ + (config_.sittingHeight - config_.targetStandingHeight);
        legControllers_[i].forceStance(footPos);
    }
}

void LocomotionEngine::updateStanding(double dt) noexcept {
    if (isTransitioning_ && transitionToStanding_) {
        transitionProgress_ += dt / std::max(1e-4, config_.standingTransitionTimeSec);
        if (transitionProgress_ >= 1.0) {
            transitionProgress_ = 1.0;
            isTransitioning_ = false;
        }

        const double s = 0.5 * (1.0 - std::cos(std::numbers::pi * transitionProgress_));

        for (size_t i = 0; i < 6; ++i) {
            Point3D footPos = nominalFeetInBody_[i];
            const double nominalZ = nominalFeetInBody_[i].z;
            const double sitZ = nominalZ + (config_.sittingHeight - config_.targetStandingHeight);
            footPos.z = sitZ + s * (nominalZ - sitZ);
            legControllers_[i].forceStance(footPos);
        }
        return;
    }

    // Check if motion command is active
    const double linSpeed = cmdLinearVel_.norm();
    const double angSpeed = std::abs(cmdAngularVel_.z);
    if (linSpeed > config_.idleVelocityThreshold || angSpeed > config_.idleVelocityThreshold) {
        state_ = LocomotionState::kLocomotion;
        idleTimer_ = 0.0;
        gaitScheduler_.reset();
        for (size_t i = 0; i < 6; ++i) {
            if (gaitScheduler_.isSwing(i)) {
                const Point3D measuredNominal = legs_[i]
                    ? previousEffPose_.applyBodyFk(legs_[i]->getCurrentFootInBodyFrame())
                    : legControllers_[i].getStancePositionBodyFrame();
                const Point3D target = gaitScheduler_.calculateTouchdownTarget(nominalFeetInBody_[i], cmdLinearVel_, cmdAngularVel_);
                legControllers_[i].initiateSwing(measuredNominal, target, gaitScheduler_.getSwingDuration());
            }
        }
        return;
    }

    // Fixed standing: estimate terrain from all 6 stance feet and maintain attitude + CoM compensation
    std::array<Point3D, 6> stanceFeetNominal{};
    size_t stanceCount = 0;
    for (size_t i = 0; i < 6; ++i) {
        if (legControllers_[i].isStance() && legs_[i]) {
            stanceFeetNominal[stanceCount++] = legControllers_[i].getCurrentFootBodyFrame();
        }
    }

    latestTerrainEstimate_ = terrainEstimator_.estimate(
        std::span<const Point3D>(stanceFeetNominal.data(), stanceCount), imuRoll_, imuPitch_, imuYaw_);

    BodyPoseCommand effPose = userBodyPose_;
    if (latestTerrainEstimate_.valid) {
        effPose.roll += latestTerrainEstimate_.attitudeCompensation.roll;
        effPose.pitch += latestTerrainEstimate_.attitudeCompensation.pitch;
        effPose.x += latestTerrainEstimate_.comShift.x;
        effPose.y += latestTerrainEstimate_.comShift.y;
    }

    previousEffPose_ = effPose;

    for (size_t i = 0; i < 6; ++i) {
        if (legs_[i]) {
            const Point3D targetBody = effPose.applyBodyIk(legControllers_[i].getStancePositionBodyFrame());
            legs_[i]->setTargetInBodyFrame(targetBody);
        }
    }
}

void LocomotionEngine::updateLocomotion(double dt) noexcept {
    // Check if velocity command is zero for automatic stop return
    const double linSpeed = cmdLinearVel_.norm();
    const double angSpeed = std::abs(cmdAngularVel_.z);
    const bool isCmdZero = (linSpeed <= config_.idleVelocityThreshold && angSpeed <= config_.idleVelocityThreshold);

    // Clean stop without teleportation:
    // Check per-leg horizontal distance only (ignore dz so bumps/holes/slopes do not trigger infinite stepping)
    constexpr double kMaxCenteringDist = 0.002;  // 2 mm
    constexpr double kMaxCenteringDistSq = kMaxCenteringDist * kMaxCenteringDist;
    std::array<bool, 6> legNeedsCentering{false, false, false, false, false, false};
    bool allLegsInStance = true;
    bool anyLegNeedsCentering = false;

    if (isCmdZero) {
        for (size_t i = 0; i < 6; ++i) {
            if (legControllers_[i].isStance()) {
                const Point3D curStance = legControllers_[i].getStancePositionBodyFrame();
                const double dx = curStance.x - nominalFeetInBody_[i].x;
                const double dy = curStance.y - nominalFeetInBody_[i].y;
                if ((dx * dx + dy * dy) > kMaxCenteringDistSq) {
                    legNeedsCentering[i] = true;
                    anyLegNeedsCentering = true;
                }
            } else {
                allLegsInStance = false;
            }
        }
    }

    if (isCmdZero && allLegsInStance && !anyLegNeedsCentering) {
        idleTimer_ += dt;
        if (idleTimer_ >= config_.idleSettlingTimeSec) {
            state_ = LocomotionState::kStanding;
            return;
        }
    } else {
        idleTimer_ = 0.0;
    }

    // Effective velocities: zero if stopping to center legs towards neutral stance
    const Point3D activeLinVel = isCmdZero ? Point3D{0.0, 0.0, 0.0} : cmdLinearVel_;
    const Point3D activeAngVel = isCmdZero ? Point3D{0.0, 0.0, 0.0} : cmdAngularVel_;

    // 1. Advance continuous gait scheduler
    gaitScheduler_.update(dt);

    // 2. Update each leg controller in nominal frame
    for (size_t i = 0; i < 6; ++i) {
        const Point3D measuredNominal = legs_[i]
            ? previousEffPose_.applyBodyFk(legs_[i]->getCurrentFootInBodyFrame())
            : legControllers_[i].getStancePositionBodyFrame();

        const bool allowLegLiftoff = !isCmdZero || legNeedsCentering[i];
        if (allowLegLiftoff && gaitScheduler_.hasLiftoffOccurred(i)) {
            const Point3D startNominal = legControllers_[i].getStancePositionBodyFrame();
            const Point3D target = gaitScheduler_.calculateTouchdownTarget(nominalFeetInBody_[i], activeLinVel, activeAngVel);
            legControllers_[i].initiateSwing(startNominal, target, gaitScheduler_.getSwingDuration());
        }

        const Point3D vStance = gaitScheduler_.calculateStanceVelocity(activeLinVel, activeAngVel,
                                                                      legControllers_[i].getStancePositionBodyFrame());
        const Point3D plannedTouchdown = gaitScheduler_.calculateTouchdownTarget(nominalFeetInBody_[i], activeLinVel, activeAngVel);
        legControllers_[i].update(dt, measuredNominal, vStance, plannedTouchdown, gaitScheduler_.getSwingDuration());
    }

    // 3. Terrain estimation from current stance feet in nominal frame (zero heap allocation)
    std::array<Point3D, 6> stanceFeetNominal{};
    size_t stanceCount = 0;
    for (size_t i = 0; i < 6; ++i) {
        if (legControllers_[i].isStance() && legs_[i]) {
            stanceFeetNominal[stanceCount++] = legControllers_[i].getCurrentFootBodyFrame();
        }
    }

    latestTerrainEstimate_ = terrainEstimator_.estimate(
        std::span<const Point3D>(stanceFeetNominal.data(), stanceCount), imuRoll_, imuPitch_, imuYaw_);

    // 4. Posture superposition: user command + terrain tilt compensation + CoM slope shift
    BodyPoseCommand effPose = userBodyPose_;
    if (latestTerrainEstimate_.valid) {
        effPose.roll += latestTerrainEstimate_.attitudeCompensation.roll;
        effPose.pitch += latestTerrainEstimate_.attitudeCompensation.pitch;
        effPose.x += latestTerrainEstimate_.comShift.x;
        effPose.y += latestTerrainEstimate_.comShift.y;
    }

    // Store effPose for inverse transformation of measured feet in next cycle
    previousEffPose_ = effPose;

    // Apply combined body posture transform to commanded leg targets once per leg
    for (size_t i = 0; i < 6; ++i) {
        if (legs_[i]) {
            const Point3D targetBody = effPose.applyBodyIk(legControllers_[i].getTargetFootBodyFrame());
            legs_[i]->setTargetInBodyFrame(targetBody);
        }
    }
}

}  // namespace hexapod::core
