#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

#include "hexapod_core/robot/hexapod_geometry.hpp"

namespace hexapod::core {
namespace {

constexpr double kEpsilon = 1e-6;

TEST(HexapodGeometryTest, DefaultDimensions) {
    const LegDimensions dims = HexapodGeometry::getDefaultLegDimensions();
    EXPECT_DOUBLE_EQ(dims.coxaLength, 0.045);
    EXPECT_DOUBLE_EQ(dims.femurLength, 0.090);
    EXPECT_DOUBLE_EQ(dims.tibiaLength, 0.120);
}

TEST(HexapodGeometryTest, ChassisRadiusConsistency) {
    const auto mountings = HexapodGeometry::getAllLegMountings();
    ASSERT_EQ(mountings.size(), 6);

    for (size_t i = 0; i < 6; ++i) {
        const Point3D& origin = mountings[i].originBodyFrame;
        const double radius = std::hypot(origin.x, origin.y);
        EXPECT_NEAR(radius, HexapodGeometry::kChassisRadius, kEpsilon);
        EXPECT_NEAR(origin.z, 0.0, kEpsilon);
    }
}

TEST(HexapodGeometryTest, FrontEdgeSymmetryAndCameraPosition) {
    const LegMounting fl = HexapodGeometry::getLegMounting(LegId::kFrontLeft);
    const LegMounting fr = HexapodGeometry::getLegMounting(LegId::kFrontRight);

    // X coordinates should be equal
    EXPECT_NEAR(fl.originBodyFrame.x, fr.originBodyFrame.x, kEpsilon);
    // Y coordinates should be opposite
    EXPECT_NEAR(fl.originBodyFrame.y, -fr.originBodyFrame.y, kEpsilon);

    // Midpoint of front edge should be at y = 0
    const double midY = (fl.originBodyFrame.y + fr.originBodyFrame.y) / 2.0;
    const double midX = (fl.originBodyFrame.x + fr.originBodyFrame.x) / 2.0;
    EXPECT_NEAR(midY, 0.0, kEpsilon);
    EXPECT_NEAR(midX, HexapodGeometry::kChassisRadius * std::cos(std::numbers::pi / 6.0), kEpsilon);
}

TEST(HexapodGeometryTest, BilateralSymmetry) {
    const auto mountings = HexapodGeometry::getAllLegMountings();

    // FL (0) vs FR (5)
    EXPECT_NEAR(mountings[0].originBodyFrame.x, mountings[5].originBodyFrame.x, kEpsilon);
    EXPECT_NEAR(mountings[0].originBodyFrame.y, -mountings[5].originBodyFrame.y, kEpsilon);

    // ML (1) vs MR (4)
    EXPECT_NEAR(mountings[1].originBodyFrame.x, mountings[4].originBodyFrame.x, kEpsilon);
    EXPECT_NEAR(mountings[1].originBodyFrame.y, -mountings[4].originBodyFrame.y, kEpsilon);

    // RL (2) vs RR (3)
    EXPECT_NEAR(mountings[2].originBodyFrame.x, mountings[3].originBodyFrame.x, kEpsilon);
    EXPECT_NEAR(mountings[2].originBodyFrame.y, -mountings[3].originBodyFrame.y, kEpsilon);
}

TEST(HexapodGeometryTest, AngularSeparationSixtyDegrees) {
    const auto mountings = HexapodGeometry::getAllLegMountings();
    const double expectedDeltaAngle = std::numbers::pi / 3.0;  // 60 deg

    EXPECT_NEAR(mountings[1].yawRad - mountings[0].yawRad, expectedDeltaAngle, kEpsilon);
    EXPECT_NEAR(mountings[2].yawRad - mountings[1].yawRad, expectedDeltaAngle, kEpsilon);
    EXPECT_NEAR(mountings[4].yawRad - mountings[3].yawRad, expectedDeltaAngle, kEpsilon);
    EXPECT_NEAR(mountings[5].yawRad - mountings[4].yawRad, expectedDeltaAngle, kEpsilon);
}

}  // namespace
}  // namespace hexapod::core

