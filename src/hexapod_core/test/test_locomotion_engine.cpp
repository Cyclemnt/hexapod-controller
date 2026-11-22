#include <gtest/gtest.h>

#include <cmath>

#include "hexapod_core/locomotion/locomotion_engine.hpp"

namespace hexapod::core {
namespace {

constexpr double kEpsilon = 1e-4;

// Helper to simulate ideal servo tracking in software unit tests
void simulateIdealJointTracking(LocomotionEngine& engine, size_t excludeGroundContactLeg = 6) {
    for (size_t j = 0; j < 6; ++j) {
        auto leg = engine.getLeg(j);
        const auto& lc = engine.getLegController(j);

        double effort = 0.0;
        // On normal flat ground, when swing reaches touchdown duration or enters probing, simulate ground contact reaction
        if (j != excludeGroundContactLeg && (lc.isProbing() || (lc.isSwing() &&
            lc.getSwingElapsedTime() >= lc.getTrajectoryGenerator().getSwingDuration() - 0.03))) {
            effort = 2.0;
        }

        leg->getCoxaJoint()->setPresentState(leg->getCoxaJoint()->getTargetAngle(), 0.0, 0.0);
        leg->getFemurJoint()->setPresentState(leg->getFemurJoint()->getTargetAngle(), 0.0, effort);
        leg->getTibiaJoint()->setPresentState(leg->getTibiaJoint()->getTargetAngle(), 0.0, effort);
    }
}

TEST(LocomotionEngineTest, StateTransitionSittingToStanding) {
    auto engine = LocomotionEngine::createDefault();

    // Start in sitting
    engine->requestSitting();
    engine->update(0.01);
    simulateIdealJointTracking(*engine);
    EXPECT_TRUE(engine->isSitting());

    // Request standing
    engine->requestStanding();
    EXPECT_TRUE(engine->isStanding());

    // Simulate transition over 2.0 seconds
    constexpr double dt = 0.02;
    for (int i = 0; i < 110; ++i) {
        engine->update(dt);
        simulateIdealJointTracking(*engine);
    }

    EXPECT_TRUE(engine->isStanding());

    // Verify all legs reached nominal standing height
    for (size_t i = 0; i < 6; ++i) {
        const auto foot = engine->getLegController(i).getStancePositionBodyFrame();
        EXPECT_NEAR(foot.z, engine->getNominalFootInBody(i).z, kEpsilon);
        EXPECT_TRUE(engine->getLegController(i).isStance());
    }
}

TEST(LocomotionEngineTest, LocomotionActivationWithCmdVelAndTripodGait) {
    auto engine = LocomotionEngine::createDefault();

    // Settle in standing
    engine->requestStanding();
    for (int i = 0; i < 110; ++i) {
        engine->update(0.02);
        simulateIdealJointTracking(*engine);
    }
    EXPECT_TRUE(engine->isStanding());

    // Send forward and lateral velocity command: vx = 0.10 m/s, vy = 0.05 m/s, wz = 0.2 rad/s
    engine->setVelocityCommand(Point3D{0.10, 0.05, 0.0}, Point3D{0.0, 0.0, 0.2});

    // One update triggers locomotion state
    engine->update(0.02);
    simulateIdealJointTracking(*engine);
    EXPECT_TRUE(engine->isLocomoting());

    // Step into first half of gait cycle (t = 0.30 s)
    for (int i = 0; i < 15; ++i) {
        engine->update(0.02);
        simulateIdealJointTracking(*engine);
    }

    // Three legs should be in stance, three in swing
    int stanceCount = 0;
    int swingCount = 0;
    for (size_t i = 0; i < 6; ++i) {
        if (engine->getLegController(i).isStance()) {
            ++stanceCount;
        } else {
            ++swingCount;
        }
    }
    EXPECT_EQ(stanceCount, 3);
    EXPECT_EQ(swingCount, 3);

    // Step into second half of gait cycle (t = 0.80 s)
    for (int i = 0; i < 25; ++i) {
        engine->update(0.02);
        simulateIdealJointTracking(*engine);
    }

    stanceCount = 0;
    swingCount = 0;
    for (size_t i = 0; i < 6; ++i) {
        if (engine->getLegController(i).isStance()) {
            ++stanceCount;
        } else {
            ++swingCount;
        }
    }
    EXPECT_EQ(stanceCount, 3);
    EXPECT_EQ(swingCount, 3);
}

TEST(LocomotionEngineTest, BumpDetectionEarlyTouchdown) {
    auto engine = LocomotionEngine::createDefault();

    // Settle in standing
    engine->requestStanding();
    for (int i = 0; i < 110; ++i) {
        engine->update(0.02);
        simulateIdealJointTracking(*engine);
    }

    // Command forward motion
    engine->setVelocityCommand(Point3D{0.15, 0.0, 0.0}, Point3D{0.0, 0.0, 0.0});
    engine->update(0.02);
    simulateIdealJointTracking(*engine);
    EXPECT_TRUE(engine->isLocomoting());

    // Find a swinging leg
    size_t swingLegIndex = 6;
    for (size_t i = 0; i < 6; ++i) {
        if (engine->getLegController(i).isSwing()) {
            swingLegIndex = i;
            break;
        }
    }
    ASSERT_LT(swingLegIndex, 6);

    // Advance to descending swing phase (e.g. t = 0.35 s > apex 0.25 s)
    for (int i = 0; i < 17; ++i) {
        engine->update(0.02);
        simulateIdealJointTracking(*engine);
    }
    ASSERT_TRUE(engine->getLegController(swingLegIndex).isSwing());

    // Simulate high effort on tibia due to early contact with a bump
    auto leg = engine->getLeg(swingLegIndex);
    leg->getTibiaJoint()->setPresentState(leg->getTibiaJoint()->getTargetAngle(), 0.0, 2.0);

    // Next update step should detect touchdown
    engine->update(0.02);
    EXPECT_TRUE(engine->getLegController(swingLegIndex).isStance());
    EXPECT_TRUE(engine->getLegController(swingLegIndex).hasEarlyTouchdownOccurred());

    // Stance foot height should be at bump level (higher than nominal target)
    const double bumpHeight = engine->getLegController(swingLegIndex).getStancePositionBodyFrame().z;
    const double nominalZ = engine->getNominalFootInBody(swingLegIndex).z;
    EXPECT_GT(bumpHeight, nominalZ);
}

TEST(LocomotionEngineTest, PitHoleDetectionAndProbing) {
    auto engine = LocomotionEngine::createDefault();

    // Settle in standing
    engine->requestStanding();
    for (int i = 0; i < 110; ++i) {
        engine->update(0.02);
        simulateIdealJointTracking(*engine);
    }

    // Command forward velocity
    engine->setVelocityCommand(Point3D{0.10, 0.0, 0.0}, Point3D{0.0, 0.0, 0.0});
    engine->update(0.02);
    simulateIdealJointTracking(*engine);
    EXPECT_TRUE(engine->isLocomoting());

    // Find a swinging leg
    size_t swingLegIndex = 6;
    for (size_t i = 0; i < 6; ++i) {
        if (engine->getLegController(i).isSwing()) {
            swingLegIndex = i;
            break;
        }
    }
    ASSERT_LT(swingLegIndex, 6);

    // Advance through the entire swing duration (0.50 s = 25 steps) without contact under this leg
    for (int i = 0; i < 26; ++i) {
        engine->update(0.02);
        simulateIdealJointTracking(*engine, swingLegIndex);
    }

    // The leg should now be in kProbing
    EXPECT_TRUE(engine->getLegController(swingLegIndex).isProbing());

    // Let it probe deeper for 5 steps (0.10 s -> ~5 mm deeper)
    for (int i = 0; i < 5; ++i) {
        engine->update(0.02);
        simulateIdealJointTracking(*engine, swingLegIndex);
    }
    EXPECT_TRUE(engine->getLegController(swingLegIndex).isProbing());
    const double probingZ = engine->getLegController(swingLegIndex).getTargetFootBodyFrame().z;
    const double nominalZ = engine->getNominalFootInBody(swingLegIndex).z;
    EXPECT_LT(probingZ, nominalZ);  // Descending into hole

    // Simulate contact at the bottom of the hole
    auto leg = engine->getLeg(swingLegIndex);
    leg->getFemurJoint()->setPresentState(leg->getFemurJoint()->getTargetAngle(), 0.0, 2.0);

    engine->update(0.02);
    EXPECT_TRUE(engine->getLegController(swingLegIndex).isStance());
    EXPECT_TRUE(engine->getLegController(swingLegIndex).hasEarlyTouchdownOccurred());
    // Stance locked at hole bottom depth
    EXPECT_LT(engine->getLegController(swingLegIndex).getStancePositionBodyFrame().z, nominalZ);
}

TEST(LocomotionEngineTest, FrontalObstacleReflexAndReplanning) {
    auto engine = LocomotionEngine::createDefault();

    // Settle in standing
    engine->requestStanding();
    for (int i = 0; i < 110; ++i) {
        engine->update(0.02);
        simulateIdealJointTracking(*engine);
    }

    // Command forward velocity
    engine->setVelocityCommand(Point3D{0.15, 0.0, 0.0}, Point3D{0.0, 0.0, 0.0});
    engine->update(0.02);
    simulateIdealJointTracking(*engine);
    EXPECT_TRUE(engine->isLocomoting());

    // Find swinging leg
    size_t swingLegIndex = 6;
    for (size_t i = 0; i < 6; ++i) {
        if (engine->getLegController(i).isSwing()) {
            swingLegIndex = i;
            break;
        }
    }
    ASSERT_LT(swingLegIndex, 6);

    // Advance to early ascending phase (t = 0.04 s < apex 0.25 s)
    engine->update(0.02);
    simulateIdealJointTracking(*engine);
    ASSERT_TRUE(engine->getLegController(swingLegIndex).isSwing());

    const double initialApex = engine->getLegController(swingLegIndex).getTrajectoryGenerator().getApexHeight();

    // Simulate frontal obstacle impact: high effort on coxa/femur
    auto leg = engine->getLeg(swingLegIndex);
    leg->getCoxaJoint()->setPresentState(leg->getCoxaJoint()->getTargetAngle(), 0.0, 2.5);

    // Next update should trigger kReflex and replanning
    engine->update(0.02);
    EXPECT_TRUE(engine->getLegController(swingLegIndex).isReflex());
    EXPECT_TRUE(engine->getLegController(swingLegIndex).hasObstacleReflexOccurred());

    // Replanned trajectory must have a higher apex (boosted clearance)
    const double replannedApex = engine->getLegController(swingLegIndex).getTrajectoryGenerator().getApexHeight();
    EXPECT_GT(replannedApex, initialApex);
}

TEST(LocomotionEngineTest, AutomaticReturnToStableStandingWhenIdle) {
    auto engine = LocomotionEngine::createDefault();

    // Settle in standing
    engine->requestStanding();
    for (int i = 0; i < 110; ++i) {
        engine->update(0.02);
        simulateIdealJointTracking(*engine);
    }

    // Start locomotion
    engine->setVelocityCommand(Point3D{0.10, 0.0, 0.0}, Point3D{0.0, 0.0, 0.0});
    engine->update(0.02);
    simulateIdealJointTracking(*engine);
    EXPECT_TRUE(engine->isLocomoting());

    // Walk for a few steps
    for (int i = 0; i < 30; ++i) {
        engine->update(0.02);
        simulateIdealJointTracking(*engine);
    }

    // Command zero velocity
    engine->setVelocityCommand(Point3D{0.0, 0.0, 0.0}, Point3D{0.0, 0.0, 0.0});

    // Advance until all legs finish current swings, center their feet without teleportation, and settle in standing
    for (int i = 0; i < 90; ++i) {
        engine->update(0.02);
        simulateIdealJointTracking(*engine);
    }

    EXPECT_TRUE(engine->isStanding());
    for (size_t i = 0; i < 6; ++i) {
        EXPECT_TRUE(engine->getLegController(i).isStance());
        const auto foot = engine->getLegController(i).getStancePositionBodyFrame();
        EXPECT_NEAR(foot.x, engine->getNominalFootInBody(i).x, 0.002);
        EXPECT_NEAR(foot.y, engine->getNominalFootInBody(i).y, 0.002);
        EXPECT_NEAR(foot.z, engine->getNominalFootInBody(i).z, 0.002);
    }
}

TEST(LocomotionEngineTest, RequestSittingIgnoredDuringLocomotion) {
    auto engine = LocomotionEngine::createDefault();

    // Settle in standing
    engine->requestStanding();
    for (int i = 0; i < 110; ++i) {
        engine->update(0.02);
        simulateIdealJointTracking(*engine);
    }
    EXPECT_TRUE(engine->isStanding());

    // Start locomotion
    engine->setVelocityCommand(Point3D{0.10, 0.0, 0.0}, Point3D{0.0, 0.0, 0.0});
    engine->update(0.02);
    simulateIdealJointTracking(*engine);
    EXPECT_TRUE(engine->isLocomoting());

    // Request sitting while moving: must be ignored
    engine->requestSitting();
    EXPECT_TRUE(engine->isLocomoting());
    EXPECT_FALSE(engine->isSitting());

    // Stop locomotion
    engine->setVelocityCommand(Point3D{0.0, 0.0, 0.0}, Point3D{0.0, 0.0, 0.0});
    for (int i = 0; i < 90; ++i) {
        engine->update(0.02);
        simulateIdealJointTracking(*engine);
    }
    EXPECT_TRUE(engine->isStanding());

    // Now request sitting from standing: accepted
    engine->requestSitting();
    EXPECT_TRUE(engine->isSitting());
}

TEST(LocomotionEngineTest, CustomGaitPreservedInSetConfig) {
    auto engine = LocomotionEngine::createDefault();

    LocomotionEngineConfig config{};
    config.gaitConfig.type = GaitType::kCustom;
    config.gaitConfig.dutyFactor = 0.70;
    config.gaitConfig.cyclePeriodSec = 2.0;
    config.gaitConfig.phaseOffsets = {0.0, 0.6, 0.2, 0.8, 0.4, 0.1};

    engine->setConfig(config);

    EXPECT_EQ(engine->getGaitScheduler().getConfig().type, GaitType::kCustom);
    EXPECT_DOUBLE_EQ(engine->getGaitScheduler().getConfig().dutyFactor, 0.70);
    EXPECT_DOUBLE_EQ(engine->getGaitScheduler().getConfig().cyclePeriodSec, 2.0);
    EXPECT_DOUBLE_EQ(engine->getGaitScheduler().getConfig().phaseOffsets[1], 0.6);
}

TEST(LocomotionEngineTest, StopOnBumpPreservesHeightAndEntersStanding) {
    auto engine = LocomotionEngine::createDefault();

    // Settle in standing
    engine->requestStanding();
    for (int i = 0; i < 110; ++i) {
        engine->update(0.02);
        simulateIdealJointTracking(*engine);
    }

    // Command forward motion
    engine->setVelocityCommand(Point3D{0.15, 0.0, 0.0}, Point3D{0.0, 0.0, 0.0});
    engine->update(0.02);
    simulateIdealJointTracking(*engine);
    EXPECT_TRUE(engine->isLocomoting());

    // Find a swinging leg
    size_t bumpLegIndex = 6;
    for (size_t i = 0; i < 6; ++i) {
        if (engine->getLegController(i).isSwing()) {
            bumpLegIndex = i;
            break;
        }
    }
    ASSERT_LT(bumpLegIndex, 6);

    // Advance to descending swing phase (t = 0.36 s > apex 0.25 s)
    for (int i = 0; i < 17; ++i) {
        engine->update(0.02);
        simulateIdealJointTracking(*engine);
    }
    ASSERT_TRUE(engine->getLegController(bumpLegIndex).isSwing());

    // Simulate early contact with a bump
    auto bumpLeg = engine->getLeg(bumpLegIndex);
    bumpLeg->getTibiaJoint()->setPresentState(bumpLeg->getTibiaJoint()->getTargetAngle(), 0.0, 2.0);
    engine->update(0.02);
    ASSERT_TRUE(engine->getLegController(bumpLegIndex).isStance());

    const double bumpHeight = engine->getLegController(bumpLegIndex).getStancePositionBodyFrame().z;
    const double nominalZ = engine->getNominalFootInBody(bumpLegIndex).z;
    ASSERT_GT(bumpHeight, nominalZ);

    // Command zero velocity and simulate solid bump underneath bumpLegIndex during centering
    engine->setVelocityCommand(Point3D{0.0, 0.0, 0.0}, Point3D{0.0, 0.0, 0.0});
    for (int i = 0; i < 90; ++i) {
        engine->update(0.02);
        simulateIdealJointTracking(*engine);

        // If bumpLegIndex swings to center horizontally, contact occurs as soon as target reaches bumpHeight
        const auto& lc = engine->getLegController(bumpLegIndex);
        if ((lc.isSwing() || lc.isProbing()) &&
            lc.getSwingElapsedTime() >= lc.getTrajectoryGenerator().getApexTime() &&
            lc.getTargetFootBodyFrame().z <= bumpHeight) {
            bumpLeg->getTibiaJoint()->setPresentState(bumpLeg->getTibiaJoint()->getTargetAngle(), 0.0, 2.0);
        }
    }

    // Must settle cleanly in standing without infinite stepping
    EXPECT_TRUE(engine->isStanding());
    for (size_t i = 0; i < 6; ++i) {
        EXPECT_TRUE(engine->getLegController(i).isStance());
    }

    // Adapted bump height and terrain estimation must be preserved in standing
    EXPECT_GT(engine->getLegController(bumpLegIndex).getStancePositionBodyFrame().z, nominalZ);
    EXPECT_TRUE(engine->getLatestTerrainEstimate().valid);
}

}  // namespace
}  // namespace hexapod::core
