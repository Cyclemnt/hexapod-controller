#include <memory>

#include "hexapod_controller/standing_controller_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<hexapod::controller::StandingControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
