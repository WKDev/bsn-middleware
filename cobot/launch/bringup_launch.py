from launch import LaunchDescription
from launch_ros.actions import Node
import os, socket

# use the namespace as the name of this computer
namespace = socket.gethostname().replace('-', '_') 

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cobot',
            namespace=namespace,
            executable='cobot_driver',
            name=f'cobot_driver'
        ),
        Node(
            package='cobot',
            namespace=namespace,
            executable='integration',
            name=f'integration'
        ),
    ])