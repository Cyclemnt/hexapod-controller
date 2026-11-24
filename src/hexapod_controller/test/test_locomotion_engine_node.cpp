#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "hexapod_controller/locomotion_engine_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace hexapod::controller {
namespace {

class LocomotionEngineNodeTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
    }

    static void TearDownTestSuite() {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }
};

TEST_F(LocomotionEngineNodeTest, NodeInitializationAndDefaultState) {
    rclcpp::NodeOptions options;
    options.append_parameter_override("auto_stand", true);

    auto node = std::make_shared<LocomotionEngineNode>(options);
    ASSERT_NE(node, nullptr);
    ASSERT_NE(node->getEngine(), nullptr);

    // Initial state is standing due to auto_stand
    EXPECT_TRUE(node->getEngine()->isStanding());
}

TEST_F(LocomotionEngineNodeTest, CustomGaitAndGeometryParameters) {
    rclcpp::NodeOptions options;
    options.append_parameter_override("gait_type", "wave");
    options.append_parameter_override("cycle_period_sec", 2.4);
    options.append_parameter_override("sitting_height", -0.055);
    options.append_parameter_override("standing_height", -0.115);
    options.append_parameter_override("stance_radius", 0.140);
    options.append_parameter_override("step_height", 0.040);
    options.append_parameter_override("imu_weight", 0.45);
    options.append_parameter_override("auto_stand", false);

    auto node = std::make_shared<LocomotionEngineNode>(options);
    const auto& config = node->getEngine()->getConfig();

    EXPECT_TRUE(node->getEngine()->isSitting());
    EXPECT_EQ(config.gaitConfig.type, core::GaitType::kWave);
    EXPECT_DOUBLE_EQ(config.gaitConfig.cyclePeriodSec, 2.4);
    EXPECT_DOUBLE_EQ(config.sittingHeight, -0.055);
    EXPECT_DOUBLE_EQ(config.targetStandingHeight, -0.115);
    EXPECT_DOUBLE_EQ(config.stanceRadius, 0.140);
    EXPECT_DOUBLE_EQ(config.legConfig.swingConfig.stepHeight, 0.040);
    EXPECT_DOUBLE_EQ(config.terrainConfig.imuWeight, 0.45);
}

TEST_F(LocomotionEngineNodeTest, StandAndSitServices) {
    rclcpp::NodeOptions options;
    options.append_parameter_override("auto_stand", false);

    auto node = std::make_shared<LocomotionEngineNode>(options);
    EXPECT_TRUE(node->getEngine()->isSitting());

    auto standClient = node->create_client<std_srvs::srv::Trigger>("/hexapod/stand");
    auto sitClient = node->create_client<std_srvs::srv::Trigger>("/hexapod/sit");

    // Request stand
    auto standReq = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto standFuture = standClient->async_send_request(standReq);
    rclcpp::spin_until_future_complete(node, standFuture, std::chrono::milliseconds(200));

    ASSERT_EQ(standFuture.wait_for(std::chrono::milliseconds(50)), std::future_status::ready);
    EXPECT_TRUE(standFuture.get()->success);
    EXPECT_TRUE(node->getEngine()->isStanding());

    // Request sit
    auto sitReq = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto sitFuture = sitClient->async_send_request(sitReq);
    rclcpp::spin_until_future_complete(node, sitFuture, std::chrono::milliseconds(200));

    ASSERT_EQ(sitFuture.wait_for(std::chrono::milliseconds(50)), std::future_status::ready);
    EXPECT_TRUE(sitFuture.get()->success);
    EXPECT_TRUE(node->getEngine()->isSitting());
}

TEST_F(LocomotionEngineNodeTest, CmdVelAndWatchdogTimeout) {
    rclcpp::NodeOptions options;
    options.append_parameter_override("cmd_vel_timeout_sec", 0.10);
    options.append_parameter_override("standing_transition_time_sec", 0.001);
    options.append_parameter_override("auto_stand", true);

    auto node = std::make_shared<LocomotionEngineNode>(options);
    auto pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    auto jsPub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    // Publie un retour /joint_states réaliste (joints indexés leg_0..leg_5)
    auto cmdSub = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/forward_position_controller/commands", 10,
        [&node, &jsPub](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            if (msg->data.size() != 18) {
                return;
            }
            sensor_msgs::msg::JointState js;
            js.effort.resize(18, 0.0);
            for (size_t leg = 0; leg < 6; ++leg) {
                const std::string prefix = "leg_" + std::to_string(leg) + "_";
                js.name.push_back(prefix + "coxa_joint");
                js.name.push_back(prefix + "femur_joint");
                js.name.push_back(prefix + "tibia_joint");

                const auto& lc = node->getEngine()->getLegController(leg);
                double effort = 0.0;
                if (lc.isProbing() || (lc.isSwing() &&
                    lc.getSwingElapsedTime() >= lc.getTrajectoryGenerator().getSwingDuration() - 0.005)) {
                    effort = 2.0;
                }
                // Signes URDF : femur = +1.0, tibia = -1.0
                js.effort[leg * 3 + 1] = effort;
                js.effort[leg * 3 + 2] = -effort;
            }
            js.position = msg->data;
            jsPub->publish(js);
        });

    // Spin to let node complete transition to standing
    for (int i = 0; i < 5; ++i) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_TRUE(node->getEngine()->isStanding());

    // Publish cmd_vel
    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0.12;
    twist.linear.y = -0.05;
    twist.angular.z = 0.25;
    pub->publish(twist);

    for (int i = 0; i < 5; ++i) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    EXPECT_TRUE(node->getEngine()->isLocomoting());

    // Wait past the 0.10 s watchdog timeout without publishing new commands
    std::this_thread::sleep_for(std::chrono::milliseconds(150));

    for (int i = 0; i < 5; ++i) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Wait for centering cycles to complete and settle back in standing
    const auto timeout = std::chrono::steady_clock::now() + std::chrono::milliseconds(3000);
    while (std::chrono::steady_clock::now() < timeout && !node->getEngine()->isStanding()) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    EXPECT_TRUE(node->getEngine()->isStanding());
}

TEST_F(LocomotionEngineNodeTest, JointStatesFeedbackMapping) {
    rclcpp::NodeOptions options;
    auto node = std::make_shared<LocomotionEngineNode>(options);
    auto pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    sensor_msgs::msg::JointState js;
    js.name = {"leg_2_coxa_joint", "leg_2_femur_joint", "leg_2_tibia_joint"};
    // URDF multipliers: coxa -1.0, femur +1.0, tibia -1.0
    js.position = {-0.20, 0.35, -0.45};
    js.velocity = {-0.10, 0.20, -0.30};
    js.effort = {-1.2, 0.8, -2.1};
    pub->publish(js);

    for (int i = 0; i < 5; ++i) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    auto leg = node->getEngine()->getLeg(2);
    ASSERT_NE(leg, nullptr);

    constexpr double kEps = 1e-4;
    // coxa: kin = urdf / -1.0 = +0.20
    EXPECT_NEAR(leg->getCoxaJoint()->getPresentAngle(), 0.20, kEps);
    EXPECT_NEAR(leg->getCoxaJoint()->getPresentVelocity(), 0.10, kEps);
    EXPECT_NEAR(leg->getCoxaJoint()->getPresentEffort(), 1.2, kEps);

    // femur: kin = urdf / 1.0 = +0.35
    EXPECT_NEAR(leg->getFemurJoint()->getPresentAngle(), 0.35, kEps);
    EXPECT_NEAR(leg->getFemurJoint()->getPresentVelocity(), 0.20, kEps);
    EXPECT_NEAR(leg->getFemurJoint()->getPresentEffort(), 0.8, kEps);

    // tibia: kin = urdf / -1.0 = +0.45
    EXPECT_NEAR(leg->getTibiaJoint()->getPresentAngle(), 0.45, kEps);
    EXPECT_NEAR(leg->getTibiaJoint()->getPresentVelocity(), 0.30, kEps);
    EXPECT_NEAR(leg->getTibiaJoint()->getPresentEffort(), 2.1, kEps);
}

TEST_F(LocomotionEngineNodeTest, ImuOrientationUpdate) {
    rclcpp::NodeOptions options;
    options.append_parameter_override("standing_transition_time_sec", 0.001);
    options.append_parameter_override("auto_stand", true);

    auto node = std::make_shared<LocomotionEngineNode>(options);
    auto pub = node->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

    sensor_msgs::msg::Imu imu;
    const double halfRoll = 0.05;  // roll = +0.10 rad
    imu.orientation.x = std::sin(halfRoll);
    imu.orientation.y = 0.0;
    imu.orientation.z = 0.0;
    imu.orientation.w = std::cos(halfRoll);
    pub->publish(imu);

    for (int i = 0; i < 6; ++i) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }

    const auto& terrain = node->getEngine()->getLatestTerrainEstimate();
    EXPECT_TRUE(terrain.valid);
    EXPECT_LT(terrain.attitudeCompensation.roll, 0.0);
}

}  // namespace
}  // namespace hexapod::controller
