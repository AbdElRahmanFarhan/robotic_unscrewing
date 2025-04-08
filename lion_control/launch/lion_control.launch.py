import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

def generate_launch_description():
    
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="lion_robot_description", package_name="lion_moveit_config"
        )
        .robot_description(file_path="config/lion_robot_description.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(
            file_path=get_package_share_directory("lion_control")
            + "/config/motion_planning_config.yaml"
        )
        .to_moveit_configs()
    )

    moveit_cpp_node = Node(
        name="lion_control",
        package="lion_control",
        executable="lion_control_node",
        output="both",
        parameters=[moveit_config.to_dict()],
    )
    ld = generate_demo_launch(moveit_config)
    ld.add_action(moveit_cpp_node)

    return ld