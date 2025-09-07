from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='euler_to_quat',
            executable='euler_to_quat',
            name='euler_to_quat',
            output='screen'
        )
    ])
