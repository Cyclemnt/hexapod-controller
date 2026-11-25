#include "hexapod_controller/locomotion_engine_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numbers>

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

LocomotionEngineNode::LocomotionEngineNode(const rclcpp::NodeOptions& options)
    : Node("locomotion_engine_node", options) {
    initParameters();
    initEngine();
    initInterfaces();

    RCLCPP_INFO(get_logger(),
                "LocomotionEngineNode initialized successfully at %.1f Hz (gait: %s, cycle: %.2fs).",
                controlRateHz_, gaitType_.c_str(), cyclePeriodSec_);
}

void LocomotionEngineNode::initParameters() {
    // Control loop & topics
    controlRateHz_ = declare_parameter<double>("control_rate_hz", 50.0);
    controlRateHz_ = declare_parameter<double>("update_rate_hz", controlRateHz_);

    commandTopic_ = declare_parameter<std::string>("command_topic", "/forward_position_controller/commands");
    cmdVelTopic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    jointStatesTopic_ = declare_parameter<std::string>("joint_states_topic", "/joint_states");
    imuTopic_ = declare_parameter<std::string>("imu_topic", "/imu/data");
    bodyPoseTopic_ = declare_parameter<std::string>("body_pose_topic", "/hexapod/body_pose_cmd");
    bodyTwistTopic_ = declare_parameter<std::string>("body_twist_topic", "/hexapod/body_twist_cmd");
    cmdVelTimeoutSec_ = declare_parameter<double>("cmd_vel_timeout_sec", 0.5);

    // Stance & posture geometry
    sittingHeight_ = declare_parameter<double>("sitting_height", -0.060);
    standingHeight_ = declare_parameter<double>("standing_height", -0.100859);
    stanceRadius_ = declare_parameter<double>("stance_radius", 0.135);
    standingTransitionTimeSec_ = declare_parameter<double>("standing_transition_time_sec", 2.5);
    autoStand_ = declare_parameter<bool>("auto_stand", true);

    // Gait parameters
    gaitType_ = declare_parameter<std::string>("gait_type", "tripod");
    cyclePeriodSec_ = declare_parameter<double>("cycle_period_sec", 1.0);
    maxStepRadius_ = declare_parameter<double>("max_step_radius", 0.035);
    stepHeight_ = declare_parameter<double>("step_height", 0.030);
    probeVelocity_ = declare_parameter<double>("probe_velocity", 0.050);
    maxProbingDist_ = declare_parameter<double>("max_probing_distance", 0.030);
    stepHeightBoost_ = declare_parameter<double>("step_height_boost", 0.025);

    // Proprioceptive contact detection thresholds
    effortThreshold_ = declare_parameter<double>("effort_threshold", 1.0);
    criticalEffortThreshold_ = declare_parameter<double>("critical_effort_threshold", 1.8);
    trackingErrorThresholdRad_ = declare_parameter<double>("tracking_error_threshold_rad", 0.06);
    obstacleEffortThreshold_ = declare_parameter<double>("obstacle_effort_threshold", 1.5);

    // Terrain estimation & IMU fusion
    imuWeight_ = declare_parameter<double>("imu_weight", 0.30);
    maxTiltAngleRad_ = declare_parameter<double>("max_tilt_angle_rad", 0.35);
    maxComShiftM_ = declare_parameter<double>("max_com_shift_m", 0.035);

    // Joint mapping multipliers
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

void LocomotionEngineNode::initEngine() {
    core::LocomotionEngineConfig config{};
    config.sittingHeight = sittingHeight_;
    config.targetStandingHeight = standingHeight_;
    config.stanceRadius = stanceRadius_;
    config.standingTransitionTimeSec = standingTransitionTimeSec_;

    if (gaitType_ == "quadruped") {
        config.gaitConfig = core::GaitConfig::createQuadruped(cyclePeriodSec_);
    } else if (gaitType_ == "wave") {
        config.gaitConfig = core::GaitConfig::createWave(cyclePeriodSec_);
    } else {
        config.gaitConfig = core::GaitConfig::createTripod(cyclePeriodSec_);
    }

    config.legConfig.swingConfig.stepHeight = stepHeight_;
    config.gaitConfig.maxStepRadius = maxStepRadius_;
    config.legConfig.swingConfig.probeVelocity = probeVelocity_;
    config.legConfig.swingConfig.maxProbingDistance = maxProbingDist_;
    config.legConfig.stepHeightBoost = stepHeightBoost_;

    config.legConfig.contactConfig.effortThreshold = effortThreshold_;
    config.legConfig.contactConfig.criticalEffortThreshold = criticalEffortThreshold_;
    config.legConfig.contactConfig.trackingErrorThresholdRad = trackingErrorThresholdRad_;
    config.legConfig.contactConfig.obstacleEffortThreshold = obstacleEffortThreshold_;

    config.terrainConfig.nominalHeight = std::abs(standingHeight_);
    config.terrainConfig.imuWeight = imuWeight_;
    config.terrainConfig.maxTiltAngleRad = maxTiltAngleRad_;
    config.terrainConfig.maxComShiftMeters = maxComShiftM_;

    engine_ = core::LocomotionEngine::createDefault(config);

    if (autoStand_) {
        engine_->requestStanding();
    } else {
        engine_->requestSitting();
    }
}

void LocomotionEngineNode::initInterfaces() {
    commandPublisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(commandTopic_, 10);

    cmdVelSubscriber_ = create_subscription<geometry_msgs::msg::Twist>(
        cmdVelTopic_, 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) { cmdVelCallback(msg); });

    jointStatesSubscriber_ = create_subscription<sensor_msgs::msg::JointState>(
        jointStatesTopic_, 20,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { jointStatesCallback(msg); });

    imuSubscriber_ = create_subscription<sensor_msgs::msg::Imu>(
        imuTopic_, 20,
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) { imuCallback(msg); });

    bodyPoseSubscriber_ = create_subscription<geometry_msgs::msg::Pose>(
        bodyPoseTopic_, 10,
        [this](const geometry_msgs::msg::Pose::SharedPtr msg) { bodyPoseCallback(msg); });

    bodyTwistSubscriber_ = create_subscription<geometry_msgs::msg::Twist>(
        bodyTwistTopic_, 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) { bodyTwistCallback(msg); });

    standService_ = create_service<std_srvs::srv::Trigger>(
        "/hexapod/stand",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            handleStandService(req, res);
        });

    sitService_ = create_service<std_srvs::srv::Trigger>(
        "/hexapod/sit",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            handleSitService(req, res);
        });

    const auto timerPeriod = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(1.0, controlRateHz_)));
    timer_ = rclcpp::create_timer(this, get_clock(), timerPeriod, [this]() { timerCallback(); });
}

void LocomotionEngineNode::timerCallback() {
    const auto currentTime = now();
    const double dt = (lastTimerCall_.nanoseconds() > 0)
                          ? (currentTime - lastTimerCall_).seconds()
                          : (1.0 / std::max(1.0, controlRateHz_));
    lastTimerCall_ = currentTime;

    // Safety watchdog: reset velocity commands if cmd_vel stream timed out
    if (hasReceivedCmdVel_) {
        const double elapsedSinceCmd = (currentTime - lastCmdVelTime_).seconds();
        if (elapsedSinceCmd > cmdVelTimeoutSec_ || elapsedSinceCmd < 0.0) {
            engine_->setVelocityCommand(core::Point3D{0.0, 0.0, 0.0}, core::Point3D{0.0, 0.0, 0.0});
            hasReceivedCmdVel_ = false;
        }
    }

    // Advance engine update by dt (clamped to realistic bounds to protect kinematics from lag)
    const double clampedDt = std::clamp(dt, 0.001, 0.100);
    engine_->update(clampedDt);

    // Collect kinematic target angles for all 6 legs
    std::array<core::LegJointAngles, 6> kinematicAngles{};
    for (size_t i = 0; i < 6; ++i) {
        kinematicAngles[i] = engine_->getLeg(i)->getTargetJointAngles();
    }

    // Map kinematic angles to URDF joint order and signs
    const auto urdfCommandValues = jointMapper_.mapToUrdf(kinematicAngles);

    std_msgs::msg::Float64MultiArray commandMsg;
    commandMsg.data = urdfCommandValues;
    commandPublisher_->publish(commandMsg);
}

void LocomotionEngineNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) || !std::isfinite(msg->angular.z)) {
        return;
    }

    lastCmdVelTime_ = now();
    hasReceivedCmdVel_ = true;

    // Linear translation (vx, vy in m/s) and yaw rotation (wz in rad/s)
    const core::Point3D linearVel{msg->linear.x, msg->linear.y, 0.0};
    const core::Point3D angularVel{0.0, 0.0, msg->angular.z};
    engine_->setVelocityCommand(linearVel, angularVel);
}

void LocomotionEngineNode::jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    const size_t nameCount = msg->name.size();
    const bool hasPositions = msg->position.size() == nameCount;
    const bool hasVelocities = msg->velocity.size() == nameCount;
    const bool hasEfforts = msg->effort.size() == nameCount;

    for (size_t i = 0; i < nameCount; ++i) {
        const int jointIdx = UrdfJointMapper::findJointIndex(msg->name[i]);
        if (jointIdx < 0) {
            continue;
        }

        const double urdfPos = hasPositions ? msg->position[i] : 0.0;
        double urdfVel = hasVelocities ? msg->velocity[i] : 0.0;
        double urdfEff = hasEfforts ? msg->effort[i] : 0.0;

        if (!std::isfinite(urdfPos)) {
            continue;
        }
        if (!std::isfinite(urdfVel)) {
            urdfVel = 0.0;
        }
        if (!std::isfinite(urdfEff)) {
            urdfEff = 0.0;
        }

        const size_t legIndex = static_cast<size_t>(jointIdx / 3);
        const size_t jointInLeg = static_cast<size_t>(jointIdx % 3);

        const double kinPos = jointMapper_.mapPositionFromUrdf(legIndex, jointInLeg, urdfPos);
        const double kinVel = jointMapper_.mapVelocityFromUrdf(legIndex, jointInLeg, urdfVel);
        const double kinEff = jointMapper_.mapEffortFromUrdf(legIndex, jointInLeg, urdfEff);

        auto leg = engine_->getLeg(legIndex);
        if (!leg) {
            continue;
        }

        switch (jointInLeg) {
            case 0:
                if (auto coxa = leg->getCoxaJoint()) {
                    coxa->setPresentState(kinPos, kinVel, kinEff);
                }
                break;
            case 1:
                if (auto femur = leg->getFemurJoint()) {
                    femur->setPresentState(kinPos, kinVel, kinEff);
                }
                break;
            case 2:
                if (auto tibia = leg->getTibiaJoint()) {
                    tibia->setPresentState(kinPos, kinVel, kinEff);
                }
                break;
            default:
                break;
        }
    }
}

void LocomotionEngineNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (!std::isfinite(msg->orientation.x) || !std::isfinite(msg->orientation.y) ||
        !std::isfinite(msg->orientation.z) || !std::isfinite(msg->orientation.w)) {
        return;
    }

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    quaternionToEuler(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w,
                      roll, pitch, yaw);
    engine_->setImuOrientation(roll, pitch, yaw);
}

void LocomotionEngineNode::bodyPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    quaternionToEuler(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w,
                      roll, pitch, yaw);

    core::BodyPoseCommand poseCmd{};
    poseCmd.x = std::clamp(msg->position.x, -0.050, 0.050);
    poseCmd.y = std::clamp(msg->position.y, -0.050, 0.050);
    poseCmd.z = std::clamp(msg->position.z, -0.040, 0.040);
    poseCmd.roll = std::clamp(roll, -0.35, 0.35);
    poseCmd.pitch = std::clamp(pitch, -0.35, 0.35);
    poseCmd.yaw = std::clamp(yaw, -0.40, 0.40);

    currentBodyPose_ = poseCmd;
    engine_->setBodyPoseCommand(poseCmd);
}

void LocomotionEngineNode::bodyTwistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    currentBodyPose_.x = std::clamp(currentBodyPose_.x + msg->linear.x * 0.01, -0.050, 0.050);
    currentBodyPose_.y = std::clamp(currentBodyPose_.y + msg->linear.y * 0.01, -0.050, 0.050);
    currentBodyPose_.z = std::clamp(currentBodyPose_.z + msg->linear.z * 0.01, -0.040, 0.040);
    currentBodyPose_.roll = std::clamp(currentBodyPose_.roll + msg->angular.x * 0.02, -0.35, 0.35);
    currentBodyPose_.pitch = std::clamp(currentBodyPose_.pitch + msg->angular.y * 0.02, -0.35, 0.35);
    currentBodyPose_.yaw = std::clamp(currentBodyPose_.yaw + msg->angular.z * 0.02, -0.40, 0.40);

    engine_->setBodyPoseCommand(currentBodyPose_);
}

void LocomotionEngineNode::handleStandService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    engine_->requestStanding();
    response->success = true;
    response->message = "Standing transition requested.";
    RCLCPP_INFO(get_logger(), "Service /hexapod/stand called: standing transition requested.");
}

void LocomotionEngineNode::handleSitService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    if (engine_->isLocomoting()) {
        response->success = false;
        response->message = "Cannot sit down while locomoting. Stop the robot first.";
        RCLCPP_WARN(get_logger(), "Service /hexapod/sit rejected: robot is locomoting.");
        return;
    }
    engine_->requestSitting();
    response->success = true;
    response->message = "Sitting transition requested.";
    RCLCPP_INFO(get_logger(), "Service /hexapod/sit called: sitting transition requested.");
}

}  // namespace hexapod::controller
