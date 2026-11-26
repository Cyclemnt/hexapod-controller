#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "hexapod_core/locomotion/gait_scheduler.hpp"

namespace hexapod::core {
namespace {

constexpr double kEpsilon = 1e-6;

TEST(GaitSchedulerTest, DefaultTripodInitialization) {
    const GaitScheduler scheduler{};

    EXPECT_NEAR(scheduler.getDutyFactor(), 0.50, kEpsilon);
    EXPECT_NEAR(scheduler.getCyclePeriod(), 1.0, kEpsilon);
    EXPECT_NEAR(scheduler.getStanceDuration(), 0.50, kEpsilon);
    EXPECT_NEAR(scheduler.getSwingDuration(), 0.50, kEpsilon);
    EXPECT_NEAR(scheduler.getMasterPhase(), 0.0, kEpsilon);

    // Group A (0, 2, 4) should start in stance (phase = 0.0)
    EXPECT_TRUE(scheduler.isStance(0));
    EXPECT_TRUE(scheduler.isStance(2));
    EXPECT_TRUE(scheduler.isStance(4));
    EXPECT_FALSE(scheduler.isSwing(0));

    // Group B (1, 3, 5) should start in swing (phase = 0.5 >= dutyFactor 0.5)
    EXPECT_TRUE(scheduler.isSwing(1));
    EXPECT_TRUE(scheduler.isSwing(3));
    EXPECT_TRUE(scheduler.isSwing(5));
    EXPECT_FALSE(scheduler.isStance(1));
}

TEST(GaitSchedulerTest, QuadrupedAndWavePresets) {
    GaitScheduler scheduler{};

    // Quadruped: dutyFactor = 2/3
    scheduler.setGait(GaitType::kQuadruped, 1.5);
    EXPECT_NEAR(scheduler.getDutyFactor(), 2.0 / 3.0, kEpsilon);
    EXPECT_NEAR(scheduler.getCyclePeriod(), 1.5, kEpsilon);
    EXPECT_NEAR(scheduler.getStanceDuration(), 1.0, kEpsilon);
    EXPECT_NEAR(scheduler.getSwingDuration(), 0.5, kEpsilon);

    // Wave: dutyFactor = 5/6
    scheduler.setGait(GaitType::kWave, 3.0);
    EXPECT_NEAR(scheduler.getDutyFactor(), 5.0 / 6.0, kEpsilon);
    EXPECT_NEAR(scheduler.getCyclePeriod(), 3.0, kEpsilon);
    EXPECT_NEAR(scheduler.getStanceDuration(), 2.5, kEpsilon);
    EXPECT_NEAR(scheduler.getSwingDuration(), 0.5, kEpsilon);
}

TEST(GaitSchedulerTest, PhaseProgressionAndContinuity) {
    GaitScheduler scheduler(GaitConfig::createTripod(1.0));

    // Update by 0.25 s
    scheduler.update(0.25);
    EXPECT_NEAR(scheduler.getMasterPhase(), 0.25, kEpsilon);
    EXPECT_NEAR(scheduler.getLegPhase(0), 0.25, kEpsilon);
    EXPECT_NEAR(scheduler.getLegPhase(1), 0.75, kEpsilon);

    // Stance progress for leg 0: 0.25 / 0.5 = 0.5
    EXPECT_NEAR(scheduler.getNormalizedStanceProgress(0), 0.50, kEpsilon);
    EXPECT_NEAR(scheduler.getNormalizedSwingProgress(0), 0.0, kEpsilon);

    // Swing progress for leg 1: (0.75 - 0.5) / 0.5 = 0.5
    EXPECT_NEAR(scheduler.getNormalizedSwingProgress(1), 0.50, kEpsilon);
    EXPECT_NEAR(scheduler.getNormalizedStanceProgress(1), 1.0, kEpsilon);

    // Advance to 1.10 s (wrap around master phase 1.0 -> 0.10)
    scheduler.update(0.85);
    EXPECT_NEAR(scheduler.getMasterPhase(), 0.10, kEpsilon);
    EXPECT_NEAR(scheduler.getLegPhase(0), 0.10, kEpsilon);
    EXPECT_NEAR(scheduler.getLegPhase(1), 0.60, kEpsilon);
}

TEST(GaitSchedulerTest, EventFlagsLiftoffAndTouchdown) {
    GaitScheduler scheduler(GaitConfig::createTripod(1.0));

    // Leg 0 starts at phase 0.0 (stance). Update by 0.49 -> phase 0.49 (still stance)
    scheduler.update(0.49);
    EXPECT_FALSE(scheduler.hasLiftoffOccurred(0));
    EXPECT_FALSE(scheduler.hasTouchdownOccurred(0));

    // Update by 0.02 -> phase 0.51 (crossed dutyFactor 0.5 -> liftoff!)
    scheduler.update(0.02);
    EXPECT_TRUE(scheduler.hasLiftoffOccurred(0));
    EXPECT_FALSE(scheduler.hasTouchdownOccurred(0));

    // Next step: liftoff flag must clear (one-shot event)
    scheduler.update(0.02);
    EXPECT_FALSE(scheduler.hasLiftoffOccurred(0));

    // Advance until touchdown: from 0.53 to 1.01 (wraps to 0.01 -> touchdown!)
    scheduler.update(0.48);
    EXPECT_TRUE(scheduler.hasTouchdownOccurred(0));
    EXPECT_FALSE(scheduler.hasLiftoffOccurred(0));

    // Next step: touchdown flag clears
    scheduler.update(0.02);
    EXPECT_FALSE(scheduler.hasTouchdownOccurred(0));
}

TEST(GaitSchedulerTest, StanceVelocityOmnidirectionalAndRotational) {
    auto cfg = GaitConfig::createTripod(1.0);
    cfg.maxStepRadius = 0.10;  // Unclamped for exact linearity check
    const GaitScheduler scheduler(cfg);

    const Point3D footPos{0.10, 0.20, -0.12};

    // 1. Pure linear motion: v_foot = -v_lin
    const Point3D vLin{0.15, -0.05, 0.0};
    const Point3D vAngZero{0.0, 0.0, 0.0};
    const Point3D vFootLin = scheduler.calculateStanceVelocity(vLin, vAngZero, footPos);
    EXPECT_NEAR(vFootLin.x, -0.15, kEpsilon);
    EXPECT_NEAR(vFootLin.y, 0.05, kEpsilon);
    EXPECT_NEAR(vFootLin.z, 0.0, kEpsilon);

    // 2. Pure yaw rotation: v_foot = -(omega x r)
    // omega = (0, 0, 0.5). r = (0.10, 0.20, -0.12).
    // omega x r = (-0.5 * 0.20, 0.5 * 0.10, 0) = (-0.10, 0.05, 0)
    // v_foot = -(-0.10, 0.05, 0) = (+0.10, -0.05, 0)
    const Point3D vLinZero{0.0, 0.0, 0.0};
    const Point3D vAngYaw{0.0, 0.0, 0.5};
    const Point3D vFootYaw = scheduler.calculateStanceVelocity(vLinZero, vAngYaw, footPos);
    EXPECT_NEAR(vFootYaw.x, 0.10, kEpsilon);
    EXPECT_NEAR(vFootYaw.y, -0.05, kEpsilon);
    EXPECT_NEAR(vFootYaw.z, 0.0, kEpsilon);

    // 3. Superposition: linear + yaw
    const Point3D vFootCombined = scheduler.calculateStanceVelocity(vLin, vAngYaw, footPos);
    EXPECT_NEAR(vFootCombined.x, vFootLin.x + vFootYaw.x, kEpsilon);
    EXPECT_NEAR(vFootCombined.y, vFootLin.y + vFootYaw.y, kEpsilon);
    EXPECT_NEAR(vFootCombined.z, 0.0, kEpsilon);
}

TEST(GaitSchedulerTest, TouchdownTargetPlacement) {
    auto cfg = GaitConfig::createTripod(1.0);
    cfg.maxStepRadius = 0.10;
    const GaitScheduler scheduler(cfg);
    // T_stance = 0.5 s

    const Point3D neutralFoot{0.15, 0.10, -0.12};
    const Point3D vLin{0.20, 0.0, 0.0};
    const Point3D vAngZero{0.0, 0.0, 0.0};

    // Forward speed 0.20 m/s over 0.5 s stance: full stroke = 0.10 m.
    // Half stroke reach ahead = +0.05 m.
    const Point3D target = scheduler.calculateTouchdownTarget(neutralFoot, vLin, vAngZero);
    EXPECT_NEAR(target.x, neutralFoot.x + 0.05, kEpsilon);
    EXPECT_NEAR(target.y, neutralFoot.y, kEpsilon);
    EXPECT_NEAR(target.z, neutralFoot.z, kEpsilon);

    // With default maxStepRadius = 0.035, reach is clamped to +0.035 m
    const GaitScheduler clampedScheduler(GaitConfig::createTripod(1.0));
    const Point3D clampedTarget = clampedScheduler.calculateTouchdownTarget(neutralFoot, vLin, vAngZero);
    EXPECT_NEAR(clampedTarget.x, neutralFoot.x + 0.035, kEpsilon);
}

}  // namespace
}  // namespace hexapod::core
