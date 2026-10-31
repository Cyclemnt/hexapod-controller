#include "hexapod_core/robot/leg.hpp"

namespace hexapod::core {

Leg::Leg(uint8_t id, const LegMounting& mounting, std::unique_ptr<ILegKinematics> kinematics,
         std::shared_ptr<IJoint> coxaJoint, std::shared_ptr<IJoint> femurJoint, std::shared_ptr<IJoint> tibiaJoint) noexcept
    : id_(id),
      mounting_(mounting),
      mountingTransform_(mounting.getTransform()),
      kinematics_(std::move(kinematics)),
      coxaJoint_(std::move(coxaJoint)),
      femurJoint_(std::move(femurJoint)),
      tibiaJoint_(std::move(tibiaJoint)) {}

bool Leg::setTargetInLegFrame(const Point3D& targetInLegFrame) {
    if (!kinematics_) {
        return false;
    }

    const auto angles = kinematics_->inverseKinematics(targetInLegFrame);
    if (!angles.has_value()) {
        return false;
    }

    targetFootInLegFrame_ = targetInLegFrame;
    targetFootInBodyFrame_ = mountingTransform_.transformPoint(targetInLegFrame);

    if (coxaJoint_) {
        coxaJoint_->setTargetAngle(angles->coxa);
    }
    if (femurJoint_) {
        femurJoint_->setTargetAngle(angles->femur);
    }
    if (tibiaJoint_) {
        tibiaJoint_->setTargetAngle(angles->tibia);
    }

    return true;
}

bool Leg::setTargetInBodyFrame(const Point3D& targetInBodyFrame) {
    const Point3D targetInLegFrame = mountingTransform_.inverseTransformPoint(targetInBodyFrame);
    return setTargetInLegFrame(targetInLegFrame);
}

Point3D Leg::getCurrentFootInLegFrame() const {
    if (!kinematics_) {
        return Point3D{};
    }
    return kinematics_->forwardKinematics(getPresentJointAngles());
}

Point3D Leg::getCurrentFootInBodyFrame() const {
    const Point3D footInLeg = getCurrentFootInLegFrame();
    return mountingTransform_.transformPoint(footInLeg);
}

LegJointAngles Leg::getTargetJointAngles() const noexcept {
    return LegJointAngles{
        .coxa = coxaJoint_ ? coxaJoint_->getTargetAngle() : 0.0,
        .femur = femurJoint_ ? femurJoint_->getTargetAngle() : 0.0,
        .tibia = tibiaJoint_ ? tibiaJoint_->getTargetAngle() : 0.0
    };
}

LegJointAngles Leg::getPresentJointAngles() const noexcept {
    return LegJointAngles{
        .coxa = coxaJoint_ ? coxaJoint_->getPresentAngle() : 0.0,
        .femur = femurJoint_ ? femurJoint_->getPresentAngle() : 0.0,
        .tibia = tibiaJoint_ ? tibiaJoint_->getPresentAngle() : 0.0
    };
}

}  // namespace hexapod::core

