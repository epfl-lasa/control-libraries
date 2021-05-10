from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os.path

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_examples',
            namespace='',
            executable='task_space_control_loop',
            name='task_space_control_loop'
        ),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('ros2_examples'), 'rviz', 'config_file.rviz')]
        )
    ])