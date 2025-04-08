#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/robot_state/robot_state.hpp>

using std::placeholders::_1;

class LionControlNode {
public:
    LionControlNode(rclcpp::Node::SharedPtr node)
        : node_(node),
          move_group_(node_, "screwdriver_group")
    {
        sub1_ = node_->create_subscription<std_msgs::msg::String>("/lion_control/go_to_named_target", 10, 
            std::bind(&LionControlNode::goToNamedTarget, this, _1));

        sub2_ = node_->create_subscription<std_msgs::msg::String>("/lion_control/move_cartesian", 10, 
            std::bind(&LionControlNode::moveCartesianCallback, this, _1));
    }

private:
    void goToNamedTarget(const std_msgs::msg::String::SharedPtr msg) 
    {
        RCLCPP_INFO(node_->get_logger(), "Received command: '%s'", msg->data.c_str());
        move_group_.setNamedTarget("home");
        moveit::core::MoveItErrorCode result = move_group_.move();
    }

    void moveCartesianCallback(const std_msgs::msg::String::SharedPtr msg) 
    {
        RCLCPP_INFO(node_->get_logger(), "Received command: '%s'", msg->data.c_str());
        geometry_msgs::msg::Pose goal;
        goal.position.x = 1.763;
        goal.position.y = 0;
        goal.position.z = 1.8;
        goal.orientation.x = 0;
        goal.orientation.y = 0;
        goal.orientation.z = 0;
        goal.orientation.w = 1;
        std::vector<geometry_msgs::msg::Pose> path;
        path.push_back(goal);

        bool success = moveCartesian(path, "base_link");
        RCLCPP_INFO(node_->get_logger(), "Cartesian Motion: '%d'", success);
        RCLCPP_INFO(node_->get_logger(), "Received command: '%s'", msg->data.c_str());
    }   
    

    bool moveCartesian(const std::vector<geometry_msgs::msg::Pose> path,
        std::string frame_id,
        bool avoid_collisions = true, 
        double eef_step = 0.001)
    {
        move_group_.setPoseReferenceFrame(frame_id);

        moveit_msgs::msg::RobotTrajectory trajectory;
        
        move_group_.computeCartesianPath(path, eef_step, trajectory, avoid_collisions);

        moveit_msgs::msg::RobotTrajectory filtered_trajectory = filterTrajectory(trajectory);
        moveit::core::MoveItErrorCode result = move_group_.execute(filtered_trajectory);

        return result == moveit::core::MoveItErrorCode::SUCCESS;
    }

    moveit_msgs::msg::RobotTrajectory filterTrajectory(moveit_msgs::msg::RobotTrajectory trajectory)
    {
        moveit_msgs::msg::RobotTrajectory filtered_trajectory;
        
        if (trajectory.joint_trajectory.points.empty()) {
            return filtered_trajectory;
        }
        
        filtered_trajectory.joint_trajectory.header = trajectory.joint_trajectory.header;
        filtered_trajectory.joint_trajectory.joint_names = trajectory.joint_trajectory.joint_names;
        
        filtered_trajectory.joint_trajectory.points.push_back(trajectory.joint_trajectory.points[0]);
        double last_time_step = trajectory.joint_trajectory.points[0].time_from_start.sec + 
        trajectory.joint_trajectory.points[0].time_from_start.nanosec * 1e-9;;
        
        for (size_t i = 1; i < trajectory.joint_trajectory.points.size(); ++i) {
            const trajectory_msgs::msg::JointTrajectoryPoint& point = trajectory.joint_trajectory.points[i];
            if ((point.time_from_start.sec + point.time_from_start.nanosec * 1e-9) > last_time_step) {
                filtered_trajectory.joint_trajectory.points.push_back(point);
                last_time_step = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
            }
        }
        
        return filtered_trajectory;
    }
    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterface move_group_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub1_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub2_;

};

int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("lion_control_node");
    auto lion_control = LionControlNode(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}