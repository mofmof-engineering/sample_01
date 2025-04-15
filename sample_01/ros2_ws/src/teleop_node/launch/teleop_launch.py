from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_node',
            executable='teleop',
            name='teleop_node',
            output='screen'
        )
    ])

