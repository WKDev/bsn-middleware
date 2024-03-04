from launch import LaunchDescription
from launch_ros.actions import Node
import os
namespace = 'rack_1'

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rack',
            namespace=namespace,
            executable='rs485_driver',
            name=f'rs485_driver'
        ),
        Node(
            package='rack',
            namespace=namespace,
            executable='gpio_driver',
            name=f'gpio_driver'
        ),
        Node(
            package='rack',
            namespace=namespace,
            executable='env_manager',
            name=f'env_manager'
        ),
    ])