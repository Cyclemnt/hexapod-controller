#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <numbers>

#include "hexapod_core/kinematics/analytical_leg_kinematics.hpp"
#include "hexapod_core/robot/hexapod_geometry.hpp"

namespace hexapod::core {
namespace {

constexpr double kEpsilon = 1e-6;

class AnalyticalLegKinematicsTest : public ::testing::Test {
protected:
    void SetUp() override {
        dimensions_ = HexapodGeometry::getDefaultLegDimensions();
        kinematics_ = std::make_unique<AnalyticalLegKinematics>(dimensions_);
    }

    LegDimensions dimensions_;
    std::unique_ptr<AnalyticalLegKinematics> kinematics_;
};

TEST_F(AnalyticalLegKinematicsTest, ForwardKinematicsNominalStandingPose) {
    // At q1=0, q2=0, q3=0: femur horizontal (+X), tibia drops by tibiaFootZ with offset +tibiaFootX
    const LegJointAngles angles{.coxa = 0.0, .femur = 0.0, .tibia = 0.0};
    const Point3D foot = kinematics_->forwardKinematics(angles);

    const double expectedX = dimensions_.coxaLength + dimensions_.femurLength + dimensions_.tibiaFootX;
    EXPECT_NEAR(foot.x, expectedX, kEpsilon);
    EXPECT_NEAR(foot.y, 0.0, kEpsilon);
    EXPECT_NEAR(foot.z, -dimensions_.tibiaFootZ, kEpsilon);
}

TEST_F(AnalyticalLegKinematicsTest, ForwardKinematicsHorizontalTibia) {
    // q1=0, q2=0, q3=+pi/2 -> tibia rotated +90 deg forward: +tibiaFootZ along +X, +tibiaFootX along +Z
    const LegJointAngles angles{.coxa = 0.0, .femur = 0.0, .tibia = std::numbers::pi / 2.0};
    const Point3D foot = kinematics_->forwardKinematics(angles);

    const double expectedX = dimensions_.coxaLength + dimensions_.femurLength + dimensions_.tibiaFootZ;
    EXPECT_NEAR(foot.x, expectedX, kEpsilon);
    EXPECT_NEAR(foot.y, 0.0, kEpsilon);
    EXPECT_NEAR(foot.z, dimensions_.tibiaFootX, kEpsilon);
}

TEST_F(AnalyticalLegKinematicsTest, ForwardKinematicsWithCoxaRotation) {
    const double coxaAngle = std::numbers::pi / 4.0;
    const LegJointAngles angles{.coxa = coxaAngle, .femur = 0.0, .tibia = 0.0};
    const Point3D foot = kinematics_->forwardKinematics(angles);

    const double expectedRadius = dimensions_.coxaLength + dimensions_.femurLength + dimensions_.tibiaFootX;
    EXPECT_NEAR(foot.x, expectedRadius * std::cos(coxaAngle), kEpsilon);
    EXPECT_NEAR(foot.y, expectedRadius * std::sin(coxaAngle), kEpsilon);
    EXPECT_NEAR(foot.z, -dimensions_.tibiaFootZ, kEpsilon);
}

TEST_F(AnalyticalLegKinematicsTest, InverseKinematicsStandingPoseRoundtrip) {
    const Point3D target{
        dimensions_.coxaLength + dimensions_.femurLength + dimensions_.tibiaFootX,
        0.0,
        -dimensions_.tibiaFootZ
    };
    const auto anglesOpt = kinematics_->inverseKinematics(target);

    ASSERT_TRUE(anglesOpt.has_value());
    EXPECT_NEAR(anglesOpt->coxa, 0.0, kEpsilon);
    EXPECT_NEAR(anglesOpt->femur, 0.0, kEpsilon);
    EXPECT_NEAR(anglesOpt->tibia, 0.0, kEpsilon);

    const Point3D footCheck = kinematics_->forwardKinematics(*anglesOpt);
    EXPECT_NEAR(footCheck.x, target.x, kEpsilon);
    EXPECT_NEAR(footCheck.y, target.y, kEpsilon);
    EXPECT_NEAR(footCheck.z, target.z, kEpsilon);
}

TEST_F(AnalyticalLegKinematicsTest, InverseKinematicsFkRoundtripMultiplePoses) {
    const double coxaAngles[] = {-0.5, -0.2, 0.0, 0.2, 0.5};
    const double femurAngles[] = {-0.3, -0.1, 0.0, 0.1, 0.3};
    const double tibiaAngles[] = {-0.4, -0.2, 0.0, 0.2, 0.4};

    for (double q1 : coxaAngles) {
        for (double q2 : femurAngles) {
            for (double q3 : tibiaAngles) {
                const LegJointAngles originalAngles{.coxa = q1, .femur = q2, .tibia = q3};
                const Point3D foot = kinematics_->forwardKinematics(originalAngles);

                const auto solvedAngles = kinematics_->inverseKinematics(foot);
                ASSERT_TRUE(solvedAngles.has_value());

                const Point3D footRecomputed = kinematics_->forwardKinematics(*solvedAngles);
                EXPECT_NEAR(footRecomputed.x, foot.x, kEpsilon);
                EXPECT_NEAR(footRecomputed.y, foot.y, kEpsilon);
                EXPECT_NEAR(footRecomputed.z, foot.z, kEpsilon);

                EXPECT_NEAR(solvedAngles->coxa, originalAngles.coxa, kEpsilon);
                EXPECT_NEAR(solvedAngles->femur, originalAngles.femur, kEpsilon);
                EXPECT_NEAR(solvedAngles->tibia, originalAngles.tibia, kEpsilon);
            }
        }
    }
}

TEST_F(AnalyticalLegKinematicsTest, UnreachableTargets) {
    const Point3D outOfReachTarget{0.350, 0.0, 0.0};
    EXPECT_FALSE(kinematics_->isReachable(outOfReachTarget));
    EXPECT_FALSE(kinematics_->inverseKinematics(outOfReachTarget).has_value());

    const Point3D tooCloseTarget{dimensions_.coxaLength, 0.0, 0.0};
    EXPECT_FALSE(kinematics_->isReachable(tooCloseTarget));
    EXPECT_FALSE(kinematics_->inverseKinematics(tooCloseTarget).has_value());
}

TEST_F(AnalyticalLegKinematicsTest, ZeroOffsetTibiaMatchesStraightGeometry) {
    const LegDimensions straightDims{
        .coxaLength = 0.045,
        .femurLength = 0.090,
        .tibiaFootZ = 0.120,
        .tibiaFootX = 0.0
    };
    const AnalyticalLegKinematics straightKinematics(straightDims);

    const Point3D foot = straightKinematics.forwardKinematics(LegJointAngles{0.0, 0.0, 0.0});
    EXPECT_NEAR(foot.x, 0.135, kEpsilon);
    EXPECT_NEAR(foot.y, 0.0, kEpsilon);
    EXPECT_NEAR(foot.z, -0.120, kEpsilon);
}

}  // namespace
}  // namespace hexapod::core