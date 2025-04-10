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

#include "lion_msgs/action/set_goal_robot.hpp"
#include "lion_msgs/msg/goal_type.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class LionControlNode {
public:
    using SetGoalRobot = lion_msgs::action::SetGoalRobot;
    using GoalHandle = rclcpp_action::ServerGoalHandle<SetGoalRobot>;

    explicit LionControlNode(rclcpp::Node::SharedPtr node)
        : node_(node),
          move_group_(node_, "arm_group")
    {
        using namespace std::placeholders;
        action_server_ = rclcpp_action::create_server<SetGoalRobot>(
            node_->get_node_base_interface(),
            node_->get_node_clock_interface(),
            node_->get_node_logging_interface(),
            node_->get_node_waitables_interface(),
          "/lion_control/go_to_goal",
          std::bind(&LionControlNode::handle_goal, this, _1, _2),
          std::bind(&LionControlNode::handle_cancel, this, _1),
          std::bind(&LionControlNode::handle_accepted, this, _1));
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

private:
    moveit::core::MoveItErrorCode moveStraight(const geometry_msgs::msg::PoseStamped goal,
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

        return result;
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
    

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const SetGoalRobot::Goal> goal)
    {
      (void)uuid;
      RCLCPP_INFO(node_->get_logger(), "Received moveit action goal with end effector: %s", goal->end_effector.c_str());
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
  
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
    {
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    }
  
    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
      using namespace std::placeholders;
      // this needs to return quickly to avoid blocking the executor, so spin up a new thread
      std::thread{std::bind(&LionControlNode::execute, this, _1), goal_handle}.detach();
    }
  
    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        moveit::core::MoveItErrorCode moveit_result;
        if(goal->goal_type.val == lion_msgs::msg::GoalType::STRAIGHT)
        {
            move_group_.setEndEffectorLink(goal->end_effector);
            moveit_result = moveStraight(goal->pose_goal);
        }
        else if(goal->goal_type.val == lion_msgs::msg::GoalType::NAMED)
        {
            move_group_.setNamedTarget(goal->named_goal);
            moveit_result = move_group_.move();
        }
        else if(goal->goal_type.val == lion_msgs::msg::GoalType::POSE)
        {
            move_group_.setPoseTarget(goal->pose_goal, goal->end_effector);
            moveit_result = move_group_.move();
        }
        else
            moveit_result = moveit::core::MoveItErrorCode::GOAL_STATE_INVALID;

        auto action_result = std::make_shared<SetGoalRobot::Result>();
        action_result->success = (moveit_result == moveit::core::MoveItErrorCode::SUCCESS);
        if (rclcpp::ok()) 
            goal_handle->succeed(action_result);
    }
    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterface move_group_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp_action::Server<SetGoalRobot>::SharedPtr action_server_;
};

int main(int argc, char *argv[]) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("lion_control_node");
    auto lion_control = LionControlNode(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}