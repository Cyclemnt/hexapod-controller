#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "hexapod_core/locomotion/terrain_estimator.hpp"

namespace hexapod::core {
namespace {

constexpr double kEpsilon = 1e-5;

TEST(TerrainEstimatorTest, FlatGroundPlaneFit) {
    const TerrainEstimator estimator{};

    // 6 stance feet on flat ground at z = -0.120 m
    const std::vector<Point3D> flatFeet = {
        Point3D{0.20, 0.10, -0.120},
        Point3D{0.00, 0.22, -0.120},
        Point3D{-0.20, 0.10, -0.120},
        Point3D{-0.20, -0.10, -0.120},
        Point3D{0.00, -0.22, -0.120},
        Point3D{0.20, -0.10, -0.120}
    };

    const auto estimate = estimator.estimate(flatFeet);

    EXPECT_TRUE(estimate.valid);
    EXPECT_NEAR(estimate.a, 0.0, kEpsilon);
    EXPECT_NEAR(estimate.b, 0.0, kEpsilon);
    EXPECT_NEAR(estimate.c, -0.120, kEpsilon);
    EXPECT_NEAR(estimate.terrainPitch, 0.0, kEpsilon);
    EXPECT_NEAR(estimate.terrainRoll, 0.0, kEpsilon);
    EXPECT_NEAR(estimate.comShift.x, 0.0, kEpsilon);
    EXPECT_NEAR(estimate.comShift.y, 0.0, kEpsilon);
    EXPECT_NEAR(estimate.attitudeCompensation.roll, 0.0, kEpsilon);
    EXPECT_NEAR(estimate.attitudeCompensation.pitch, 0.0, kEpsilon);
}

TEST(TerrainEstimatorTest, KnownInclinedPlaneFitAndComShift) {
    TerrainEstimatorConfig config{
        .nominalHeight = 0.120,
        .imuWeight = 0.0,  // Pure terrain fit for verification
        .maxTiltAngleRad = 0.50,
        .maxComShiftMeters = 0.050
    };
    TerrainEstimator estimator(config);

    // Plane: z = 0.08 * x - 0.06 * y - 0.120
    const double aKnown = 0.08;
    const double bKnown = -0.06;
    const double cKnown = -0.120;

    std::vector<Point3D> inclinedFeet;
    const std::vector<std::pair<double, double>> xyPositions = {
        {0.20, 0.10}, {0.00, 0.22}, {-0.20, 0.10},
        {-0.20, -0.10}, {0.00, -0.22}, {0.20, -0.10}
    };

    for (const auto& [x, y] : xyPositions) {
        const double z = aKnown * x + bKnown * y + cKnown;
        inclinedFeet.emplace_back(x, y, z);
    }

    // Robot on slope matches terrain tilt: nose up (imuPitch < 0) when a > 0
    const double imuPitch = -std::atan(aKnown);
    const double imuRoll = std::atan(bKnown);
    const auto estimate = estimator.estimate(inclinedFeet, imuRoll, imuPitch, 0.0);

    EXPECT_TRUE(estimate.valid);
    EXPECT_NEAR(estimate.a, aKnown, kEpsilon);
    EXPECT_NEAR(estimate.b, bKnown, kEpsilon);
    EXPECT_NEAR(estimate.c, cKnown, kEpsilon);

    // REP-103: terrainPitch = -atan(a), terrainRoll = +atan(b)
    const double expectedPitch = -std::atan(aKnown);
    const double expectedRoll = std::atan(bKnown);
    EXPECT_NEAR(estimate.terrainPitch, expectedPitch, kEpsilon);
    EXPECT_NEAR(estimate.terrainRoll, expectedRoll, kEpsilon);

    // CoM horizontal shift calculated from IMU:
    // When nose is up (imuPitch < 0), gravity pulls towards -X => comShift.x = -h * tan(imuPitch)
    // When left side is down (imuRoll < 0), gravity pulls towards +Y => comShift.y = +h * tan(imuRoll)
    const double expectedDx = -config.nominalHeight * std::tan(imuPitch);
    const double expectedDy = config.nominalHeight * std::tan(imuRoll);
    EXPECT_NEAR(estimate.comShift.x, expectedDx, kEpsilon);
    EXPECT_NEAR(estimate.comShift.y, expectedDy, kEpsilon);

    // Chassis attitude compensation to keep chassis horizontal: -fusedPitch, -fusedRoll
    EXPECT_NEAR(estimate.attitudeCompensation.pitch, -expectedPitch, kEpsilon);
    EXPECT_NEAR(estimate.attitudeCompensation.roll, -expectedRoll, kEpsilon);
}

TEST(TerrainEstimatorTest, ImuFusionWeighting) {
    TerrainEstimatorConfig config{
        .nominalHeight = 0.120,
        .imuWeight = 0.40,  // 60% terrain, 40% IMU
        .maxTiltAngleRad = 0.50,
        .maxComShiftMeters = 0.050
    };
    TerrainEstimator estimator(config);

    // Flat feet (terrain pitch = 0, roll = 0)
    const std::vector<Point3D> flatFeet = {
        Point3D{0.20, 0.10, -0.120},
        Point3D{0.00, 0.22, -0.120},
        Point3D{-0.20, 0.10, -0.120},
        Point3D{-0.20, -0.10, -0.120}
    };

    const double imuRoll = 0.10;
    const double imuPitch = -0.05;

    const auto estimate = estimator.estimate(flatFeet, imuRoll, imuPitch, 0.0);

    EXPECT_TRUE(estimate.valid);
    // Terrain has 0, so fused attitude = 0.40 * IMU
    // Attitude compensation to maintain horizontal chassis = -fused
    EXPECT_NEAR(estimate.attitudeCompensation.roll, -0.40 * imuRoll, kEpsilon);
    EXPECT_NEAR(estimate.attitudeCompensation.pitch, -0.40 * imuPitch, kEpsilon);

    // CoM shift calculated from IMU
    const double expectedDx = -config.nominalHeight * std::tan(imuPitch);
    const double expectedDy = +config.nominalHeight * std::tan(imuRoll);
    EXPECT_NEAR(estimate.comShift.x, expectedDx, kEpsilon);
    EXPECT_NEAR(estimate.comShift.y, expectedDy, kEpsilon);
}

TEST(TerrainEstimatorTest, BodyPoseCommandFkIkInversion) {
    const BodyPoseCommand pose{
        .x = 0.02,
        .y = -0.015,
        .z = 0.01,
        .roll = 0.12,
        .pitch = -0.08,
        .yaw = 0.15
    };

    const Point3D pNominal{0.18, 0.12, -0.115};
    const Point3D pBody = pose.applyBodyIk(pNominal);
    const Point3D pReconstructed = pose.applyBodyFk(pBody);

    EXPECT_NEAR(pReconstructed.x, pNominal.x, kEpsilon);
    EXPECT_NEAR(pReconstructed.y, pNominal.y, kEpsilon);
    EXPECT_NEAR(pReconstructed.z, pNominal.z, kEpsilon);
}

TEST(TerrainEstimatorTest, DegenerateCases) {
    const TerrainEstimator estimator{};

    // Case 1: Fewer than 3 points
    const std::vector<Point3D> twoFeet = {
        Point3D{0.20, 0.10, -0.120},
        Point3D{-0.20, -0.10, -0.120}
    };
    const auto estimateFew = estimator.estimate(twoFeet);
    EXPECT_FALSE(estimateFew.valid);

    // Case 2: Collinear points
    const std::vector<Point3D> collinearFeet = {
        Point3D{-0.20, 0.0, -0.120},
        Point3D{0.00, 0.0, -0.120},
        Point3D{0.20, 0.0, -0.120}
    };
    const auto estimateCollinear = estimator.estimate(collinearFeet);
    EXPECT_FALSE(estimateCollinear.valid);
}

}  // namespace
}  // namespace hexapod::core
