#ifndef HEXAPOD_CONTROLLER_STANDING_CONTROLLER_NODE_HPP_
#define HEXAPOD_CONTROLLER_STANDING_CONTROLLER_NODE_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "hexapod_controller/joint_mapping.hpp"
#include "hexapod_core/robot/hexapod_geometry.hpp"
#include "hexapod_core/robot/leg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace hexapod::controller {

struct BodyPoseCommand {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double roll{0.0};
    double pitch{0.0};
    double yaw{0.0};
};

class StandingControllerNode : public rclcpp::Node {
public:
    explicit StandingControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~StandingControllerNode() override = default;

private:
    void initParameters();
    void initLegs();
    void timerCallback();
    void bodyPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void bodyTwistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    [[nodiscard]] core::Point3D applyBodyIk(const core::Point3D& worldFoot, const BodyPoseCommand& pose) const noexcept;

    // ROS communications
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr commandPublisher_{};
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr bodyPoseSubscriber_{};
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr bodyTwistSubscriber_{};
    rclcpp::TimerBase::SharedPtr timer_{};

    // Configuration and parameters
    double targetBodyHeight_{-0.120};
    double sittingBodyHeight_{-0.060};
    double stanceRadius_{0.135};
    double transitionTimeSec_{2.5};
    double updateRateHz_{50.0};
    std::string commandTopic_{"/forward_position_controller/commands"};

    // Runtime state
    std::array<std::unique_ptr<core::Leg>, 6> legs_{};
    std::array<core::Point3D, 6> nominalFeetInWorld_{};
    UrdfJointMapper jointMapper_{};
    BodyPoseCommand currentBodyPose_{};
    BodyPoseCommand targetBodyPose_{};

    rclcpp::Time startTime_{};
    bool isStanding_{false};
};

}  // namespace hexapod::controller

#endif  // HEXAPOD_CONTROLLER_STANDING_CONTROLLER_NODE_HPP_
