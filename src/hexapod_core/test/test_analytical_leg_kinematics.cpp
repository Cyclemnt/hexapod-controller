#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

#include "hexapod_core/kinematics/analytical_leg_kinematics.hpp"

namespace hexapod::core {
namespace {

constexpr double kEpsilon = 1e-6;

class AnalyticalLegKinematicsTest : public ::testing::Test {
protected:
    void SetUp() override {
        dimensions_ = LegDimensions{.coxaLength = 0.045, .femurLength = 0.090, .tibiaLength = 0.120};
        kinematics_ = std::make_unique<AnalyticalLegKinematics>(dimensions_);
    }

    LegDimensions dimensions_;
    std::unique_ptr<AnalyticalLegKinematics> kinematics_;
};

TEST_F(AnalyticalLegKinematicsTest, ForwardKinematicsStraightArm) {
    // All angles at 0 rad: arm extended horizontally along +X
    const LegJointAngles angles{.coxa = 0.0, .femur = 0.0, .tibia = 0.0};
    const Point3D foot = kinematics_->forwardKinematics(angles);

    EXPECT_NEAR(foot.x, 0.045 + 0.090 + 0.120, kEpsilon);
    EXPECT_NEAR(foot.y, 0.0, kEpsilon);
    EXPECT_NEAR(foot.z, 0.0, kEpsilon);
}

TEST_F(AnalyticalLegKinematicsTest, ForwardKinematicsStandingPose) {
    // q1 = 0, q2 = 0, q3 = -pi/2: femur horizontal, tibia pointing straight down
    const LegJointAngles angles{.coxa = 0.0, .femur = 0.0, .tibia = -std::numbers::pi / 2.0};
    const Point3D foot = kinematics_->forwardKinematics(angles);

    EXPECT_NEAR(foot.x, 0.045 + 0.090, kEpsilon);
    EXPECT_NEAR(foot.y, 0.0, kEpsilon);
    EXPECT_NEAR(foot.z, -0.120, kEpsilon);
}

TEST_F(AnalyticalLegKinematicsTest, ForwardKinematicsWithCoxaRotation) {
    // Coxa rotated by +45 deg (pi/4), femur horizontal, tibia straight down
    const double coxaAngle = std::numbers::pi / 4.0;
    const LegJointAngles angles{.coxa = coxaAngle, .femur = 0.0, .tibia = -std::numbers::pi / 2.0};
    const Point3D foot = kinematics_->forwardKinematics(angles);

    const double expectedRadius = 0.045 + 0.090;
    EXPECT_NEAR(foot.x, expectedRadius * std::cos(coxaAngle), kEpsilon);
    EXPECT_NEAR(foot.y, expectedRadius * std::sin(coxaAngle), kEpsilon);
    EXPECT_NEAR(foot.z, -0.120, kEpsilon);
}

TEST_F(AnalyticalLegKinematicsTest, InverseKinematicsStandingPoseRoundtrip) {
    const Point3D target{0.135, 0.0, -0.120};
    const auto anglesOpt = kinematics_->inverseKinematics(target);

    ASSERT_TRUE(anglesOpt.has_value());
    EXPECT_NEAR(anglesOpt->coxa, 0.0, kEpsilon);
    EXPECT_NEAR(anglesOpt->femur, 0.0, kEpsilon);
    EXPECT_NEAR(anglesOpt->tibia, -std::numbers::pi / 2.0, kEpsilon);

    const Point3D footCheck = kinematics_->forwardKinematics(*anglesOpt);
    EXPECT_NEAR(footCheck.x, target.x, kEpsilon);
    EXPECT_NEAR(footCheck.y, target.y, kEpsilon);
    EXPECT_NEAR(footCheck.z, target.z, kEpsilon);
}

TEST_F(AnalyticalLegKinematicsTest, InverseKinematicsFkRoundtripMultiplePoses) {
    const double coxaAngles[] = {-0.5, -0.2, 0.0, 0.2, 0.5};
    const double femurAngles[] = {-0.3, -0.1, 0.0, 0.1, 0.3};
    const double tibiaAngles[] = {-2.0, -1.5, -1.0, -0.6};

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
    // Target far beyond maximum reach (45 + 90 + 120 = 255 mm)
    const Point3D outOfReachTarget{0.350, 0.0, 0.0};
    EXPECT_FALSE(kinematics_->isReachable(outOfReachTarget));
    EXPECT_FALSE(kinematics_->inverseKinematics(outOfReachTarget).has_value());

    // Target too close (min reach: |90 - 120| = 30 mm from femur base)
    const Point3D tooCloseTarget{0.045, 0.0, 0.0};
    EXPECT_FALSE(kinematics_->isReachable(tooCloseTarget));
    EXPECT_FALSE(kinematics_->inverseKinematics(tooCloseTarget).has_value());
}

}  // namespace
}  // namespace hexapod::core

