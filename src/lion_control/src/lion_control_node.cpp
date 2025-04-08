#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

class LionControlNode {
public:
    LionControlNode(rclcpp::Node::SharedPtr node)
        : node_(node),
          move_group_(node_, "screwdriver_group")
    {
        subscription_ = node_->create_subscription<std_msgs::msg::String>(
            "/lion_control/go_to_named_target", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                this->goToNamedTarget(msg);
            });
    }

private:
    void goToNamedTarget(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(node_->get_logger(), "Received command: '%s'", msg->data.c_str());

        move_group_.setNamedTarget("home");
        moveit::core::MoveItErrorCode result = move_group_.move();
    }

    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterface move_group_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("lion_control_node");
    auto lion_control = LionControlNode(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}