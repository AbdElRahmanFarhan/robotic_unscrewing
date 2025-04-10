#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using std::placeholders::_1;

class LionControlNode {
public:
    LionControlNode(rclcpp::Node::SharedPtr node)
        : node_(node),
          move_group_(node_, "arm_group")
    {
        sub1_ = node_->create_subscription<std_msgs::msg::String>("/lion_control/go_to_named_target", 1, 
            std::bind(&LionControlNode::goToNamedTarget, this, _1));

        sub2_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>("/lion_control/move_straight", 1, 
            std::bind(&LionControlNode::moveStraightCallback, this, _1));
        
        sub3_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>("/lion_control/go_to_pose_target", 1, 
            std::bind(&LionControlNode::goToPoseTarget, this, _1));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

private:
    void goToNamedTarget(const std_msgs::msg::String::SharedPtr msg) 
    {
        RCLCPP_INFO(node_->get_logger(), "Received command: '%s'", msg->data.c_str());
        move_group_.setNamedTarget("home");
        moveit::core::MoveItErrorCode result = move_group_.move();
    }
    void goToPoseTarget(const geometry_msgs::msg::PoseStamped::SharedPtr goal) 
    {
        RCLCPP_INFO(node_->get_logger(), "Received Pose Target");
        move_group_.setPoseTarget(*goal, "screwdriver_tcp");
        moveit::core::MoveItErrorCode result = move_group_.move();
    }

    void moveStraightCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal) 
    {
        RCLCPP_INFO(node_->get_logger(), "Received Move Straight Target");
        move_group_.setEndEffectorLink("screwdriver_tcp");
        RCLCPP_INFO(node_->get_logger(), "End effector Link: '%s'", move_group_.getEndEffectorLink().c_str());
        bool success = moveStraight(*goal);
        RCLCPP_INFO(node_->get_logger(), "Cartesian Motion: '%d'", success);
    }
    
    bool moveStraight(const geometry_msgs::msg::PoseStamped goal,
        bool avoid_collisions = true, 
        double eef_step = 0.001)
    {
        std::string frame_id = "base_link";
        geometry_msgs::msg::PoseStamped goal_transformed = tf_buffer_->transform(goal, frame_id);
        std::vector<geometry_msgs::msg::Pose> path;
        path.push_back(goal_transformed.pose);
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
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub2_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub3_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

};

int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("lion_control_node");
    auto lion_control = LionControlNode(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}