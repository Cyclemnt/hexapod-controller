#include "hexapod_controller/standing_controller_node.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "hexapod_core/actuator/joint.hpp"
#include "hexapod_core/kinematics/analytical_leg_kinematics.hpp"

namespace hexapod::controller {

namespace {

void quaternionToEuler(double qx, double qy, double qz, double qw, double& roll, double& pitch, double& yaw) noexcept {
    const double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    const double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    const double sinp = 2.0 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1.0) {
        pitch = std::copysign(std::numbers::pi / 2.0, sinp);
    } else {
        pitch = std::asin(sinp);
    }

    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace

StandingControllerNode::StandingControllerNode(const rclcpp::NodeOptions& options)
    : Node("standing_controller_node", options) {
    initParameters();
    initLegs();

    commandPublisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(commandTopic_, 10);

    bodyPoseSubscriber_ = create_subscription<geometry_msgs::msg::Pose>(
        "/hexapod/body_pose_cmd", 10,
        [this](const geometry_msgs::msg::Pose::SharedPtr msg) { bodyPoseCallback(msg); });

    bodyTwistSubscriber_ = create_subscription<geometry_msgs::msg::Twist>(
        "/hexapod/body_twist_cmd", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) { bodyTwistCallback(msg); });

    const auto timerPeriod = std::chrono::duration<double>(1.0 / updateRateHz_);
    startTime_ = now();
    timer_ = create_wall_timer(timerPeriod, [this]() { timerCallback(); });

    RCLCPP_INFO(get_logger(), "Standing controller node initialized at %.1f Hz. Standing up smoothly...", updateRateHz_);
}

void StandingControllerNode::initParameters() {
    targetBodyHeight_ = declare_parameter<double>("target_body_height", -0.120);
    sittingBodyHeight_ = declare_parameter<double>("sitting_body_height", -0.060);
    stanceRadius_ = declare_parameter<double>("stance_radius", 0.135);
    transitionTimeSec_ = declare_parameter<double>("transition_time_sec", 2.5);
    updateRateHz_ = declare_parameter<double>("update_rate_hz", 50.0);
    commandTopic_ = declare_parameter<std::string>("command_topic", "/forward_position_controller/commands");

    const double coxaMult = declare_parameter<double>("joint_mapping.coxa_multiplier", -1.0);
    const double femurMult = declare_parameter<double>("joint_mapping.femur_multiplier", 1.0);
    const double tibiaMult = declare_parameter<double>("joint_mapping.tibia_multiplier", -1.0);

    for (size_t i = 0; i < 6; ++i) {
        jointMapper_.setLegConfig(i, LegJointConfig{
            .coxa = JointAxisConfig{.multiplier = coxaMult, .offsetRad = 0.0},
            .femur = JointAxisConfig{.multiplier = femurMult, .offsetRad = 0.0},
            .tibia = JointAxisConfig{.multiplier = tibiaMult, .offsetRad = 0.0}
        });
    }
}

void StandingControllerNode::initLegs() {
    const auto dims = core::HexapodGeometry::getDefaultLegDimensions();
    const core::JointLimits coxaLimits{.minAngleRad = -1.100, .maxAngleRad = 1.100};
    const core::JointLimits pitchLimits{};
    const core::JointCalibration calib{};

    for (size_t i = 0; i < 6; ++i) {
        const auto mounting = core::HexapodGeometry::getLegMountingByIndex(static_cast<uint8_t>(i));
        auto kinematics = std::make_unique<core::AnalyticalLegKinematics>(dims);

        auto coxaJoint = std::make_shared<core::Joint>(static_cast<uint8_t>(i * 3 + 1), coxaLimits, calib);
        auto femurJoint = std::make_shared<core::Joint>(static_cast<uint8_t>(i * 3 + 2), pitchLimits, calib);
        auto tibiaJoint = std::make_shared<core::Joint>(static_cast<uint8_t>(i * 3 + 3), pitchLimits, calib);

        legs_[i] = std::make_unique<core::Leg>(
            static_cast<uint8_t>(i), mounting, std::move(kinematics),
            coxaJoint, femurJoint, tibiaJoint);

        const core::Point3D nominalFootInLegFrame{stanceRadius_, 0.0, targetBodyHeight_};
        nominalFeetInWorld_[i] = mounting.getTransform().transformPoint(nominalFootInLegFrame);
    }
}

void StandingControllerNode::timerCallback() {
    const double elapsedSec = (now() - startTime_).seconds();
    const double progress = std::clamp(elapsedSec / transitionTimeSec_, 0.0, 1.0);
    const double smoothProgress = 0.5 * (1.0 - std::cos(std::numbers::pi * progress));

    const double currentHeight = sittingBodyHeight_ + smoothProgress * (targetBodyHeight_ - sittingBodyHeight_);

    if (progress >= 1.0 && !isStanding_) {
        isStanding_ = true;
        RCLCPP_INFO(get_logger(), "Standing posture reached successfully! Ready for body pose adjustments.");
    }

    // Smooth filter for body pose to prevent abrupt jerks that break ground friction
    constexpr double kFilterCoeff = 0.08;
    currentBodyPose_.x += kFilterCoeff * (targetBodyPose_.x - currentBodyPose_.x);
    currentBodyPose_.y += kFilterCoeff * (targetBodyPose_.y - currentBodyPose_.y);
    currentBodyPose_.z += kFilterCoeff * (targetBodyPose_.z - currentBodyPose_.z);
    currentBodyPose_.roll += kFilterCoeff * (targetBodyPose_.roll - currentBodyPose_.roll);
    currentBodyPose_.pitch += kFilterCoeff * (targetBodyPose_.pitch - currentBodyPose_.pitch);
    currentBodyPose_.yaw += kFilterCoeff * (targetBodyPose_.yaw - currentBodyPose_.yaw);

    std::array<core::LegJointAngles, 6> kinematicAngles{};

    for (size_t i = 0; i < 6; ++i) {
        core::Point3D footWorld = nominalFeetInWorld_[i];
        footWorld.z = legs_[i]->getMounting().originBodyFrame.z + currentHeight;

        const core::Point3D footInBody = applyBodyIk(footWorld, currentBodyPose_);

        if (!legs_[i]->setTargetInBodyFrame(footInBody)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Target unreachable for leg %zu!", i);
        }

        kinematicAngles[i] = legs_[i]->getTargetJointAngles();
    }

    const auto urdfCommandValues = jointMapper_.mapToUrdf(kinematicAngles);

    std_msgs::msg::Float64MultiArray commandMsg;
    commandMsg.data = urdfCommandValues;
    commandPublisher_->publish(commandMsg);
}

core::Point3D StandingControllerNode::applyBodyIk(const core::Point3D& worldFoot, const BodyPoseCommand& pose) const noexcept {
    const double dx = worldFoot.x - pose.x;
    const double dy = worldFoot.y - pose.y;
    const double dz = worldFoot.z - pose.z;

    // R_z(-yaw)
    const double cosYaw = std::cos(-pose.yaw);
    const double sinYaw = std::sin(-pose.yaw);
    const double x1 = dx * cosYaw - dy * sinYaw;
    const double y1 = dx * sinYaw + dy * cosYaw;
    const double z1 = dz;

    // R_y(-pitch)
    const double cosPitch = std::cos(-pose.pitch);
    const double sinPitch = std::sin(-pose.pitch);
    const double x2 = x1 * cosPitch + z1 * sinPitch;
    const double y2 = y1;
    const double z2 = -x1 * sinPitch + z1 * cosPitch;

    // R_x(-roll)
    const double cosRoll = std::cos(-pose.roll);
    const double sinRoll = std::sin(-pose.roll);
    const double x3 = x2;
    const double y3 = y2 * cosRoll - z2 * sinRoll;
    const double z3 = y2 * sinRoll + z2 * cosRoll;

    return core::Point3D{x3, y3, z3};
}

void StandingControllerNode::bodyPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    if (!isStanding_) {
        return;
    }

    targetBodyPose_.x = std::clamp(msg->position.x, -0.040, 0.040);
    targetBodyPose_.y = std::clamp(msg->position.y, -0.040, 0.040);
    targetBodyPose_.z = std::clamp(msg->position.z, -0.030, 0.030);

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    quaternionToEuler(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w, roll, pitch, yaw);

    targetBodyPose_.roll = std::clamp(roll, -0.25, 0.25);
    targetBodyPose_.pitch = std::clamp(pitch, -0.25, 0.25);
    targetBodyPose_.yaw = std::clamp(yaw, -0.30, 0.30);
}

void StandingControllerNode::bodyTwistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!isStanding_) {
        return;
    }

    targetBodyPose_.x = std::clamp(targetBodyPose_.x + msg->linear.x * 0.01, -0.040, 0.040);
    targetBodyPose_.y = std::clamp(targetBodyPose_.y + msg->linear.y * 0.01, -0.040, 0.040);
    targetBodyPose_.z = std::clamp(targetBodyPose_.z + msg->linear.z * 0.01, -0.030, 0.030);

    targetBodyPose_.roll = std::clamp(targetBodyPose_.roll + msg->angular.x * 0.02, -0.25, 0.25);
    targetBodyPose_.pitch = std::clamp(targetBodyPose_.pitch + msg->angular.y * 0.02, -0.25, 0.25);
    targetBodyPose_.yaw = std::clamp(targetBodyPose_.yaw + msg->angular.z * 0.02, -0.30, 0.30);
}

}  // namespace hexapod::controller
