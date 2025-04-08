from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os 


package_share_directory = get_package_share_directory('camera')

screws_data_file = os.path.join(package_share_directory, 'config', 'screw_poses.json')

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera',
            # namespace='camera',
            executable='camera',
            name='camera',
            output='screen',
            parameters=[{'screws_data_file': screws_data_file}]
        ),
    ])