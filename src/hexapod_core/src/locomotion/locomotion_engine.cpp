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
        lastGroundFeetNominal_[i] = nominalFeetInBody_[i];
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
            lastGroundFeetNominal_[i] = footPos;
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
        stopCenteringSteps_.fill(0);
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

    // Update ground memory from stance legs
    for (size_t i = 0; i < 6; ++i) {
        if (legControllers_[i].isStance()) {
            lastGroundFeetNominal_[i] = legControllers_[i].getStancePositionBodyFrame();
        }
    }

    const double worldImuRoll = imuRoll_ - (filteredTerrainRoll_ + userBodyPose_.roll);
    const double worldImuPitch = imuPitch_ - (filteredTerrainPitch_ + userBodyPose_.pitch);

    latestTerrainEstimate_ = terrainEstimator_.estimate(
        std::span<const Point3D>(lastGroundFeetNominal_.data(), 6), worldImuRoll, worldImuPitch, imuYaw_);

    // Smooth attitude compensation with 150 ms first-order low-pass filter
    constexpr double kAttitudeTauSec = 0.15;
    const double alpha = 1.0 - std::exp(-std::max(0.0, dt) / kAttitudeTauSec);
    if (latestTerrainEstimate_.valid) {
        filteredTerrainRoll_ += alpha * (latestTerrainEstimate_.attitudeCompensation.roll - filteredTerrainRoll_);
        filteredTerrainPitch_ += alpha * (latestTerrainEstimate_.attitudeCompensation.pitch - filteredTerrainPitch_);
    }

    BodyPoseCommand effPose = userBodyPose_;
    if (latestTerrainEstimate_.valid) {
        effPose.roll += filteredTerrainRoll_;
        effPose.pitch += filteredTerrainPitch_;
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

    if (!isCmdZero) {
        stopCenteringSteps_.fill(0);
    }

    // Clean stop without leg pumping:
    // Allow at most 1 centering step per leg if horizontal offset > 2 mm
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
                if ((dx * dx + dy * dy) > kMaxCenteringDistSq && stopCenteringSteps_[i] < 1) {
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
    auto computeTerrainAwareTarget = [this, &activeLinVel, &activeAngVel](size_t legIdx) -> Point3D {
        Point3D target = gaitScheduler_.calculateTouchdownTarget(
            nominalFeetInBody_[legIdx], activeLinVel, activeAngVel);
        if (latestTerrainEstimate_.valid) {
            const double planeZ = latestTerrainEstimate_.a * target.x +
                                  latestTerrainEstimate_.b * target.y +
                                  latestTerrainEstimate_.c;
            const double nomZ = nominalFeetInBody_[legIdx].z;
            const double blendedZ = nomZ + latestTerrainEstimate_.planarityConfidence * (planeZ - nomZ);
            target.z = std::clamp(blendedZ, nomZ - 0.05, nomZ + 0.05);
        }
        return target;
    };

    for (size_t i = 0; i < 6; ++i) {
        const Point3D measuredNominal = legs_[i]
            ? previousEffPose_.applyBodyFk(legs_[i]->getCurrentFootInBodyFrame())
            : legControllers_[i].getStancePositionBodyFrame();

        const Point3D plannedTouchdown = computeTerrainAwareTarget(i);

        const bool allowLegLiftoff = !isCmdZero || legNeedsCentering[i];
        if (allowLegLiftoff && gaitScheduler_.hasLiftoffOccurred(i)) {
            const Point3D startNominal = legControllers_[i].getStancePositionBodyFrame();
            legControllers_[i].initiateSwing(startNominal, plannedTouchdown, gaitScheduler_.getSwingDuration());
        }

        const Point3D vStance = gaitScheduler_.calculateStanceVelocity(activeLinVel, activeAngVel,
                                                                      legControllers_[i].getStancePositionBodyFrame());
        legControllers_[i].update(dt, measuredNominal, vStance, plannedTouchdown, gaitScheduler_.getSwingDuration());

        if (legControllers_[i].isStance()) {
            lastGroundFeetNominal_[i] = legControllers_[i].getStancePositionBodyFrame();
        }
    }

    // 3. Regulate mean stance height smoothly toward nominal chassis-frame foot Z
    double sumStanceZ = 0.0;
    double sumNominalZ = 0.0;
    size_t stanceCount = 0;
    for (size_t i = 0; i < 6; ++i) {
        if (legControllers_[i].isStance()) {
            sumStanceZ += legControllers_[i].getStancePositionBodyFrame().z;
            sumNominalZ += nominalFeetInBody_[i].z;
            ++stanceCount;
        }
    }
    if (stanceCount > 0) {
        const double meanStanceZ = sumStanceZ / static_cast<double>(stanceCount);
        const double meanNominalZ = sumNominalZ / static_cast<double>(stanceCount);
        const double zError = std::clamp(meanNominalZ - meanStanceZ, -0.025, 0.025);
        constexpr double kHeightRecoveryRate = 2.5;  // 1/s
        const double zStep = zError * std::min(1.0, kHeightRecoveryRate * std::max(0.0, dt));
        for (size_t i = 0; i < 6; ++i) {
            if (legControllers_[i].isStance()) {
                legControllers_[i].adjustStanceHeight(zStep);
                lastGroundFeetNominal_[i] = legControllers_[i].getStancePositionBodyFrame();
            }
        }
    }

    // 4. Terrain estimation from full 6-foot support memory in nominal frame
    const double worldImuRoll = imuRoll_ - (filteredTerrainRoll_ + userBodyPose_.roll);
    const double worldImuPitch = imuPitch_ - (filteredTerrainPitch_ + userBodyPose_.pitch);

    latestTerrainEstimate_ = terrainEstimator_.estimate(
        std::span<const Point3D>(lastGroundFeetNominal_.data(), 6), worldImuRoll, worldImuPitch, imuYaw_);

    // Smooth attitude compensation with 150 ms first-order low-pass filter
    constexpr double kAttitudeTauSec = 0.15;
    const double alpha = 1.0 - std::exp(-std::max(0.0, dt) / kAttitudeTauSec);
    if (latestTerrainEstimate_.valid) {
        filteredTerrainRoll_ += alpha * (latestTerrainEstimate_.attitudeCompensation.roll - filteredTerrainRoll_);
        filteredTerrainPitch_ += alpha * (latestTerrainEstimate_.attitudeCompensation.pitch - filteredTerrainPitch_);
    }

    // 5. Posture superposition: user command + filtered terrain tilt compensation + CoM slope shift
    BodyPoseCommand effPose = userBodyPose_;
    if (latestTerrainEstimate_.valid) {
        effPose.roll += filteredTerrainRoll_;
        effPose.pitch += filteredTerrainPitch_;
        effPose.x += latestTerrainEstimate_.comShift.x;
        effPose.y += latestTerrainEstimate_.comShift.y;
    }

    previousEffPose_ = effPose;

    // 6. Apply combined body posture transform to commanded leg targets
    for (size_t i = 0; i < 6; ++i) {
        if (legs_[i]) {
            const Point3D targetBody = effPose.applyBodyIk(legControllers_[i].getTargetFootBodyFrame());
            legs_[i]->setTargetInBodyFrame(targetBody);
        }
    }
}

}  // namespace hexapod::core
