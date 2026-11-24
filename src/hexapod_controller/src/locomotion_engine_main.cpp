#include <memory>

#include "hexapod_controller/locomotion_engine_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<hexapod::controller::LocomotionEngineNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
