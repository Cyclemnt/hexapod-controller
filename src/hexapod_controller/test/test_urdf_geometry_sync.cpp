#include <gtest/gtest.h>

#include <array>
#include <cstdio>
#include <memory>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "hexapod_core/robot/hexapod_geometry.hpp"

#if __has_include(<urdf/model.hpp>)
#include <urdf/model.hpp>
#else
#include <urdf/model.h>
#endif

namespace hexapod::controller {
namespace {

constexpr double kTolerance = 1e-5;

struct PipeCloser {
    void operator()(FILE* file) const noexcept {
        if (file != nullptr) {
            pclose(file);
        }
    }
};

std::string expandXacro(const std::string& xacroPath) {
    const std::string command = "xacro \"" + xacroPath + "\"";
    std::array<char, 4096> buffer{};
    std::string output;

    const std::unique_ptr<FILE, PipeCloser> pipe(popen(command.c_str(), "r"));
    if (!pipe) {
        return {};
    }

    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe.get()) != nullptr) {
        output += buffer.data();
    }
    return output;
}

TEST(UrdfGeometrySyncTest, UrdfMatchesCoreHexapodGeometry) {
    const std::string shareDir = ament_index_cpp::get_package_share_directory("hexapod_description");
    const std::string urdfXml = expandXacro(shareDir + "/urdf/hexapod.xacro");
    ASSERT_FALSE(urdfXml.empty()) << "Failed to expand hexapod.xacro";

    urdf::Model model;
    ASSERT_TRUE(model.initString(urdfXml)) << "Failed to parse expanded URDF";

    const auto dims = core::HexapodGeometry::getDefaultLegDimensions();

    for (uint8_t i = 0; i < 6; ++i) {
        const std::string prefix = "leg_" + std::to_string(i) + "_";
        const auto mounting = core::HexapodGeometry::getLegMountingByIndex(i);

        const auto coxaJoint = model.getJoint(prefix + "coxa_joint");
        const auto femurJoint = model.getJoint(prefix + "femur_joint");
        const auto tibiaJoint = model.getJoint(prefix + "tibia_joint");
        const auto footJoint = model.getJoint(prefix + "tibia_to_foot_joint");

        ASSERT_NE(coxaJoint, nullptr);
        ASSERT_NE(femurJoint, nullptr);
        ASSERT_NE(tibiaJoint, nullptr);
        ASSERT_NE(footJoint, nullptr);

        const auto& pCoxa = coxaJoint->parent_to_joint_origin_transform.position;
        EXPECT_NEAR(pCoxa.x, mounting.originBodyFrame.x, kTolerance);
        EXPECT_NEAR(pCoxa.y, mounting.originBodyFrame.y, kTolerance);
        EXPECT_NEAR(pCoxa.z, mounting.originBodyFrame.z, kTolerance);

        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        coxaJoint->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch, yaw);
        EXPECT_NEAR(yaw, mounting.yawRad, kTolerance);

        EXPECT_NEAR(femurJoint->parent_to_joint_origin_transform.position.x, dims.coxaLength, kTolerance);
        EXPECT_NEAR(tibiaJoint->parent_to_joint_origin_transform.position.x, dims.femurLength, kTolerance);

        const auto& pFoot = footJoint->parent_to_joint_origin_transform.position;
        EXPECT_NEAR(pFoot.x, dims.tibiaFootX, kTolerance);
        EXPECT_NEAR(-pFoot.z, dims.tibiaFootZ, kTolerance);

        const auto tibiaLink = model.getLink(prefix + "tibia");
        ASSERT_NE(tibiaLink, nullptr);
        for (const auto& collision : tibiaLink->collision_array) {
            if (collision && collision->geometry && collision->geometry->type == urdf::Geometry::SPHERE) {
                const auto sphere = std::dynamic_pointer_cast<urdf::Sphere>(collision->geometry);
                ASSERT_NE(sphere, nullptr);
                EXPECT_NEAR(sphere->radius, core::HexapodGeometry::kFootRadius, kTolerance);
            }
        }
    }
}

}  // namespace
}  // namespace hexapod::controller
