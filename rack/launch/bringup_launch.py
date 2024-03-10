from launch import LaunchDescription
from launch_ros.actions import Node
import os, socket

# use the namespace as the name of this computer
namespace = socket.gethostname()

namespace.replace('-', '_')  # replace hyphens with underscores 

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
            name=f'led_manager'
        ),
    ])