#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <numbers>

#include "hexapod_core/actuator/joint.hpp"
#include "hexapod_core/kinematics/analytical_leg_kinematics.hpp"
#include "hexapod_core/robot/hexapod_geometry.hpp"
#include "hexapod_core/robot/leg.hpp"

namespace hexapod::core {
namespace {

constexpr double kEpsilon = 1e-5;

class LegTest : public ::testing::Test {
protected:
    void SetUp() override {
        const auto dimensions = HexapodGeometry::getDefaultLegDimensions();
        auto kinematics = std::make_unique<AnalyticalLegKinematics>(dimensions);

        const JointLimits limits{};
        const JointCalibration calibration{};

        coxaJoint_ = std::make_shared<Joint>(1, limits, calibration);
        femurJoint_ = std::make_shared<Joint>(2, limits, calibration);
        tibiaJoint_ = std::make_shared<Joint>(3, limits, calibration);

        // Setup Leg 0 (Front-Left: +30 deg yaw, corner at R=0.130m)
        mounting_ = HexapodGeometry::getLegMounting(LegId::kFrontLeft);

        leg_ = std::make_unique<Leg>(0, mounting_, std::move(kinematics), coxaJoint_, femurJoint_, tibiaJoint_);
    }

    LegMounting mounting_;
    std::shared_ptr<Joint> coxaJoint_;
    std::shared_ptr<Joint> femurJoint_;
    std::shared_ptr<Joint> tibiaJoint_;
    std::unique_ptr<Leg> leg_;
};

TEST_F(LegTest, ConstructionAndAccessors) {
    EXPECT_EQ(leg_->getId(), 0);
    EXPECT_EQ(leg_->getCoxaJoint()->getId(), 1);
    EXPECT_EQ(leg_->getFemurJoint()->getId(), 2);
    EXPECT_EQ(leg_->getTibiaJoint()->getId(), 3);
}

TEST_F(LegTest, SetTargetInLegFrameStandingPose) {
    const Point3D targetLegFrame{0.135, 0.0, -0.120};
    const bool success = leg_->setTargetInLegFrame(targetLegFrame);

    EXPECT_TRUE(success);
    EXPECT_NEAR(leg_->getTargetFootInLegFrame().x, targetLegFrame.x, kEpsilon);
    EXPECT_NEAR(leg_->getTargetFootInLegFrame().y, targetLegFrame.y, kEpsilon);
    EXPECT_NEAR(leg_->getTargetFootInLegFrame().z, targetLegFrame.z, kEpsilon);

    const auto angles = leg_->getTargetJointAngles();
    EXPECT_NEAR(angles.coxa, 0.0, kEpsilon);
    EXPECT_NEAR(angles.femur, 0.0, kEpsilon);
    EXPECT_NEAR(angles.tibia, -std::numbers::pi / 2.0, kEpsilon);

    EXPECT_NEAR(coxaJoint_->getTargetAngle(), 0.0, kEpsilon);
    EXPECT_NEAR(femurJoint_->getTargetAngle(), 0.0, kEpsilon);
    EXPECT_NEAR(tibiaJoint_->getTargetAngle(), -std::numbers::pi / 2.0, kEpsilon);
}

TEST_F(LegTest, SetTargetInBodyFrameMatchesLegFrame) {
    const Point3D targetLegFrame{0.135, 0.0, -0.120};
    const Point3D targetBodyFrame = mounting_.getTransform().transformPoint(targetLegFrame);

    const bool success = leg_->setTargetInBodyFrame(targetBodyFrame);
    EXPECT_TRUE(success);

    EXPECT_NEAR(leg_->getTargetFootInLegFrame().x, targetLegFrame.x, kEpsilon);
    EXPECT_NEAR(leg_->getTargetFootInLegFrame().y, targetLegFrame.y, kEpsilon);
    EXPECT_NEAR(leg_->getTargetFootInLegFrame().z, targetLegFrame.z, kEpsilon);

    EXPECT_NEAR(leg_->getTargetFootInBodyFrame().x, targetBodyFrame.x, kEpsilon);
    EXPECT_NEAR(leg_->getTargetFootInBodyFrame().y, targetBodyFrame.y, kEpsilon);
    EXPECT_NEAR(leg_->getTargetFootInBodyFrame().z, targetBodyFrame.z, kEpsilon);
}

TEST_F(LegTest, ForwardKinematicsFromPresentJointState) {
    coxaJoint_->setPresentState(0.1, 0.0, 0.0);
    femurJoint_->setPresentState(0.0, 0.0, 0.0);
    tibiaJoint_->setPresentState(-std::numbers::pi / 2.0, 0.0, 0.0);

    const Point3D footInLeg = leg_->getCurrentFootInLegFrame();
    const double expectedR = 0.045 + 0.090;
    EXPECT_NEAR(footInLeg.x, expectedR * std::cos(0.1), kEpsilon);
    EXPECT_NEAR(footInLeg.y, expectedR * std::sin(0.1), kEpsilon);
    EXPECT_NEAR(footInLeg.z, -0.120, kEpsilon);

    const Point3D footInBody = leg_->getCurrentFootInBodyFrame();
    const Point3D expectedBody = mounting_.getTransform().transformPoint(footInLeg);
    EXPECT_NEAR(footInBody.x, expectedBody.x, kEpsilon);
    EXPECT_NEAR(footInBody.y, expectedBody.y, kEpsilon);
    EXPECT_NEAR(footInBody.z, expectedBody.z, kEpsilon);
}

TEST_F(LegTest, RejectUnreachableTarget) {
    const Point3D impossibleTarget{1.0, 0.0, 0.0};
    const bool success = leg_->setTargetInLegFrame(impossibleTarget);
    EXPECT_FALSE(success);
}

}  // namespace
}  // namespace hexapod::core

