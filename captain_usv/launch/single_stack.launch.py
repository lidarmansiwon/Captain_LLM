from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='captain_usv', executable='fsm_node', name='mission_fsm', output='screen'),
        Node(package='captain_usv', executable='captain_node', name='captain_llm', output='screen'),
        Node(package='captain_usv', executable='rule_engine_node', name='rule_engine', output='screen'),
        Node(package='captain_usv', executable='coordinator_node', name='coordinator', output='screen'),
    # WorldModel 노드는 별도 패키지 또는 시뮬에서 퍼블리시
    ])