from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='captain_usv', executable='world_model_node', name='world_model', output='screen',
            parameters=[
                {'odom_topic': '/agent_0/odom'},
                {'publish_rate_hz': 10.0},
                {'constraints_pub_rate_hz': 0.5},
                # Demo target & No-Go
                {'target.enabled': True},
                {'target.pose.x': 180.0},
                {'target.pose.y': 0.0},
                {'target.pose.yaw_deg': 200.0},
                {'target.vel.u': 1.0},
                {'nogo.enabled': True},
                {'nogo.points': [[120.0,40.0],[160.0,40.0],[160.0,80.0],[120.0,80.0]]},
            ]
        )
    ])