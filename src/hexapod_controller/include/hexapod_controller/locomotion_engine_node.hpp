#ifndef HEXAPOD_CONTROLLER_LOCOMOTION_ENGINE_NODE_HPP_
#define HEXAPOD_CONTROLLER_LOCOMOTION_ENGINE_NODE_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "hexapod_controller/joint_mapping.hpp"
#include "hexapod_core/locomotion/locomotion_engine.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace hexapod::controller {

class LocomotionEngineNode : public rclcpp::Node {
public:
    explicit LocomotionEngineNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~LocomotionEngineNode() override = default;

    [[nodiscard]] const std::shared_ptr<core::LocomotionEngine>& getEngine() const noexcept { return engine_; }

private:
    void initParameters();
    void initEngine();
    void initInterfaces();

    void timerCallback();
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void bodyPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void bodyTwistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    void handleStandService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void handleSitService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // ROS communications
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr commandPublisher_{};
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSubscriber_{};
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStatesSubscriber_{};
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubscriber_{};
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr bodyPoseSubscriber_{};
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr bodyTwistSubscriber_{};

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr standService_{};
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr sitService_{};

    rclcpp::TimerBase::SharedPtr timer_{};

    // Configuration parameters
    double controlRateHz_{50.0};
    double cmdVelTimeoutSec_{0.5};
    std::string commandTopic_{"/forward_position_controller/commands"};
    std::string cmdVelTopic_{"/cmd_vel"};
    std::string jointStatesTopic_{"/joint_states"};
    std::string imuTopic_{"/imu/data"};
    std::string bodyPoseTopic_{"/hexapod/body_pose_cmd"};
    std::string bodyTwistTopic_{"/hexapod/body_twist_cmd"};

    double sittingHeight_{-0.060};
    double standingHeight_{-0.100859};
    double stanceRadius_{0.135};
    double standingTransitionTimeSec_{2.5};
    bool autoStand_{true};

    std::string gaitType_{"tripod"};
    double cyclePeriodSec_{1.0};
    double stepHeight_{0.030};
    double maxStepRadius_{0.035};
    double probeVelocity_{0.050};
    double maxProbingDist_{0.030};
    double stepHeightBoost_{0.025};

    double effortThreshold_{1.0};
    double criticalEffortThreshold_{1.8};
    double trackingErrorThresholdRad_{0.06};
    double obstacleEffortThreshold_{1.5};

    double imuWeight_{0.30};
    double maxTiltAngleRad_{0.35};
    double maxComShiftM_{0.035};

    // Runtime state
    std::shared_ptr<core::LocomotionEngine> engine_{};
    UrdfJointMapper jointMapper_{};
    core::BodyPoseCommand currentBodyPose_{};

    rclcpp::Time lastCmdVelTime_{0, 0, RCL_ROS_TIME};
    rclcpp::Time lastTimerCall_{0, 0, RCL_ROS_TIME};
    bool hasReceivedCmdVel_{false};
};

}  // namespace hexapod::controller

#endif  // HEXAPOD_CONTROLLER_LOCOMOTION_ENGINE_NODE_HPP_
