#include <gtest/gtest.h>

#include <cmath>

#include "hexapod_core/locomotion/swing_trajectory_generator.hpp"

namespace hexapod::core {
namespace {

constexpr double kTolPos = 1e-5;
constexpr double kTolVel = 1e-4;
constexpr double kTolAcc = 1e-3;

TEST(SwingTrajectoryGeneratorTest, BoundaryConditionsAtLiftoffAndTouchdown) {
    SwingConfig config{
        .stepHeight = 0.040,
        .probeVelocity = 0.060,
        .apexRatio = 0.50,
        .maxProbingDistance = 0.050
    };
    SwingTrajectoryGenerator generator(config);

    const Point3D liftPos{0.12, 0.15, -0.10};
    const Point3D touchTarget{0.18, 0.15, -0.10};
    const double duration = 0.60;

    generator.reset(liftPos, touchTarget, duration);

    // Liftoff at t = 0: position exact, velocity zero, acceleration zero
    const auto p0 = generator.evaluate(0.0);
    EXPECT_NEAR(p0.position.x, liftPos.x, kTolPos);
    EXPECT_NEAR(p0.position.y, liftPos.y, kTolPos);
    EXPECT_NEAR(p0.position.z, liftPos.z, kTolPos);
    EXPECT_NEAR(p0.velocity.x, 0.0, kTolVel);
    EXPECT_NEAR(p0.velocity.y, 0.0, kTolVel);
    EXPECT_NEAR(p0.velocity.z, 0.0, kTolVel);
    EXPECT_NEAR(p0.acceleration.x, 0.0, kTolAcc);
    EXPECT_NEAR(p0.acceleration.y, 0.0, kTolAcc);
    EXPECT_NEAR(p0.acceleration.z, 0.0, kTolAcc);

    // Touchdown at t = duration: horizontal zero, z has probing velocity, zero acceleration
    const auto pEnd = generator.evaluate(duration);
    EXPECT_NEAR(pEnd.position.x, touchTarget.x, kTolPos);
    EXPECT_NEAR(pEnd.position.y, touchTarget.y, kTolPos);
    EXPECT_NEAR(pEnd.position.z, touchTarget.z, kTolPos);
    EXPECT_NEAR(pEnd.velocity.x, 0.0, kTolVel);
    EXPECT_NEAR(pEnd.velocity.y, 0.0, kTolVel);
    EXPECT_NEAR(pEnd.velocity.z, -config.probeVelocity, kTolVel);
    EXPECT_NEAR(pEnd.acceleration.x, 0.0, kTolAcc);
    EXPECT_NEAR(pEnd.acceleration.y, 0.0, kTolAcc);
    EXPECT_NEAR(pEnd.acceleration.z, 0.0, kTolAcc);
}

TEST(SwingTrajectoryGeneratorTest, ApexClearanceAndVerticalPeak) {
    const SwingConfig config{.stepHeight = 0.035, .probeVelocity = 0.050, .apexRatio = 0.50};
    SwingTrajectoryGenerator generator(config);

    const Point3D liftPos{0.10, 0.10, -0.12};
    const Point3D touchTarget{0.16, 0.10, -0.10};
    const double duration = 0.50;

    generator.reset(liftPos, touchTarget, duration);

    // Apex height = max(-0.12, -0.10) + 0.035 = -0.065
    EXPECT_NEAR(generator.getApexHeight(), -0.065, kTolPos);

    // Evaluate at apex time t = 0.25 s
    const auto pApex = generator.evaluate(0.25);
    EXPECT_NEAR(pApex.position.z, -0.065, kTolPos);
    EXPECT_NEAR(pApex.velocity.z, 0.0, kTolVel);
    EXPECT_NEAR(pApex.acceleration.z, 0.0, kTolAcc);
}

TEST(SwingTrajectoryGeneratorTest, C2ContinuityAndNumericalDerivatives) {
    const SwingConfig config{.stepHeight = 0.040, .probeVelocity = 0.050, .apexRatio = 0.50};
    SwingTrajectoryGenerator generator(config);

    const Point3D liftPos{0.10, 0.12, -0.10};
    const Point3D touchTarget{0.20, 0.14, -0.10};
    const double duration = 0.50;
    generator.reset(liftPos, touchTarget, duration);

    constexpr double dt = 1e-5;
    constexpr int numSteps = 500;

    for (int i = 1; i < numSteps; ++i) {
        const double t = (static_cast<double>(i) / numSteps) * duration;
        const auto pt = generator.evaluate(t);
        const auto ptPlus = generator.evaluate(t + dt);
        const auto ptMinus = generator.evaluate(t - dt);

        // Check numerical velocity = (p(t+dt) - p(t-dt)) / (2*dt)
        const Point3D numVel = (ptPlus.position - ptMinus.position) / (2.0 * dt);
        EXPECT_NEAR(numVel.x, pt.velocity.x, 1e-3);
        EXPECT_NEAR(numVel.y, pt.velocity.y, 1e-3);
        EXPECT_NEAR(numVel.z, pt.velocity.z, 1e-3);

        // Check numerical acceleration = (v(t+dt) - v(t-dt)) / (2*dt)
        const Point3D numAcc = (ptPlus.velocity - ptMinus.velocity) / (2.0 * dt);
        EXPECT_NEAR(numAcc.x, pt.acceleration.x, 2e-2);
        EXPECT_NEAR(numAcc.y, pt.acceleration.y, 2e-2);
        EXPECT_NEAR(numAcc.z, pt.acceleration.z, 2e-2);
    }
}

TEST(SwingTrajectoryGeneratorTest, ProbingContinuationPastNominalTouchdown) {
    const SwingConfig config{
        .stepHeight = 0.030,
        .probeVelocity = 0.050,
        .apexRatio = 0.50,
        .maxProbingDistance = 0.040
    };
    SwingTrajectoryGenerator generator(config);

    const Point3D liftPos{0.10, 0.10, -0.10};
    const Point3D touchTarget{0.15, 0.10, -0.10};
    const double duration = 0.40;
    generator.reset(liftPos, touchTarget, duration);

    // At t = duration: nominal landing reached
    EXPECT_TRUE(generator.isProbing(duration));

    // At t = duration + 0.20 s: should descend by 0.20 * 0.050 = 0.010 m
    const auto pProbe1 = generator.evaluate(duration + 0.20);
    EXPECT_NEAR(pProbe1.position.x, touchTarget.x, kTolPos);
    EXPECT_NEAR(pProbe1.position.y, touchTarget.y, kTolPos);
    EXPECT_NEAR(pProbe1.position.z, touchTarget.z - 0.010, kTolPos);
    EXPECT_NEAR(pProbe1.velocity.z, -0.050, kTolVel);
    EXPECT_NEAR(pProbe1.acceleration.z, 0.0, kTolAcc);

    // At t = duration + 2.0 s: should be clamped at maxProbingDistance (0.040 m)
    const auto pMaxProbe = generator.evaluate(duration + 2.0);
    EXPECT_NEAR(pMaxProbe.position.z, touchTarget.z - 0.040, kTolPos);
    EXPECT_NEAR(pMaxProbe.velocity.z, 0.0, kTolVel);
}

TEST(SwingTrajectoryGeneratorTest, DynamicC2ReplanningFromInstantaneousState) {
    const SwingConfig config{.stepHeight = 0.035, .probeVelocity = 0.050, .apexRatio = 0.50};
    SwingTrajectoryGenerator generator(config);

    const Point3D liftPos{0.10, 0.10, -0.10};
    const Point3D touchTarget{0.20, 0.10, -0.10};
    const double duration = 0.50;
    generator.reset(liftPos, touchTarget, duration);

    // Mid-swing reflex at t = 0.20 s (e.g. obstacle hit)
    const double hitTime = 0.20;
    const TrajectoryPoint hitState = generator.evaluate(hitTime);

    // Replan with +25 mm step height boost to clear obstacle
    const Point3D newTarget{0.22, 0.10, -0.10};
    const double remainingDuration = 0.35;
    const double boost = 0.025;

    generator.replan(hitState, newTarget, remainingDuration, boost);

    // At t = 0 of replanned trajectory: must match hitState EXACTLY (zero jerk / jump)
    const auto startReplanned = generator.evaluate(0.0);
    EXPECT_NEAR(startReplanned.position.x, hitState.position.x, kTolPos);
    EXPECT_NEAR(startReplanned.position.y, hitState.position.y, kTolPos);
    EXPECT_NEAR(startReplanned.position.z, hitState.position.z, kTolPos);
    EXPECT_NEAR(startReplanned.velocity.x, hitState.velocity.x, kTolVel);
    EXPECT_NEAR(startReplanned.velocity.y, hitState.velocity.y, kTolVel);
    EXPECT_NEAR(startReplanned.velocity.z, hitState.velocity.z, kTolVel);
    EXPECT_NEAR(startReplanned.acceleration.x, hitState.acceleration.x, kTolAcc);
    EXPECT_NEAR(startReplanned.acceleration.y, hitState.acceleration.y, kTolAcc);
    EXPECT_NEAR(startReplanned.acceleration.z, hitState.acceleration.z, kTolAcc);

    // Apex height of replanned trajectory must incorporate the boost
    const double expectedApex = std::max(hitState.position.z, newTarget.z) + config.stepHeight + boost;
    EXPECT_NEAR(generator.getApexHeight(), expectedApex, kTolPos);

    // End of replanned trajectory must reach newTarget with probing velocity
    const auto endReplanned = generator.evaluate(remainingDuration);
    EXPECT_NEAR(endReplanned.position.x, newTarget.x, kTolPos);
    EXPECT_NEAR(endReplanned.position.y, newTarget.y, kTolPos);
    EXPECT_NEAR(endReplanned.position.z, newTarget.z, kTolPos);
    EXPECT_NEAR(endReplanned.velocity.z, -config.probeVelocity, kTolVel);
    EXPECT_NEAR(endReplanned.acceleration.z, 0.0, kTolAcc);
}

}  // namespace
}  // namespace hexapod::core
