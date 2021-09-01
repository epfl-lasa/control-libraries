from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_examples',
            namespace='',
            executable='joint_space_torque_control_loop',
            name='joint_space_torque_control_loop'
        ),
    ])