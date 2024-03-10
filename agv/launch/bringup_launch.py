from launch import LaunchDescription
from launch_ros.actions import Node
import os
namespace = 'agv_1'

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='agv',
            namespace=namespace,
            executable='api_middleware',
            name=f'api_middleware',
            output='screen'
        ),
        Node(
            package='agv',
            namespace=namespace,
            executable='teleop_gamepad',
            name=f'teleop_gamepad'
        ),
        Node(
            package='agv',
            namespace=namespace,
            executable='integration',
            name=f'integration',
            output='screen'
        ),
    ])
