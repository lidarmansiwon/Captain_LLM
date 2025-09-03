from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='rosgpt', executable='server_node', name='server'),
        Node(package='rosgpt', executable='subcaptain_node', name='subcaptain'),
        Node(package='rosgpt', executable='captain_node', name='captain'),
        Node(package='rosgpt', executable='rosgpt', name='rosgpt_gateway'),
    ])
