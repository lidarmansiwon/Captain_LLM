#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from captain_interfaces.msg import HighLevelPlan, Trajectory

class CoordinatorNode(Node):
    def __init__(self):
        super().__init__('coordinator')
        self.sub_p = self.create_subscription(HighLevelPlan, '/plan/highlevel', self.on_plan, 10)
        self.pub_t = self.create_publisher(Trajectory, '/u1/trajectory', 10)

    def on_plan(self, p: HighLevelPlan):
        traj = Trajectory()
        traj.frame = 'map'
        traj.path = p.waypoints
        # 간단: 같은 속도 적용
        traj.speed_profile = [1.5 for _ in p.waypoints]
        traj.behavior = 'intercept_then_escort'
        self.pub_t.publish(traj)

def main():
    rclpy.init(); rclpy.spin(CoordinatorNode()); rclpy.shutdown()