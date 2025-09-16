#!/usr/bin/env python3
import math
from typing import List

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry

from captain_interfaces.msg import (
    WorldState, EgoState, Pose2D, Velocity2D, Health,
    Target, Hazard, Env, Constraints, Limits, Polygon, Point2D
)


def quat_to_yaw_deg(x: float, y: float, z: float, w: float) -> float:
    """Return yaw [deg] from quaternion (x,y,z,w), ENU frame convention."""
    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(yaw)


class WorldModelNode(Node):
    def __init__(self):
        super().__init__('world_model')

        # Params — you can override via launch or CLI
        self.declare_parameter('odom_topic', '/agent_0/odom')
        self.declare_parameter('publish_rate_hz', 10.0)

        # Constraints params
        self.declare_parameter('limits.dcpa_min_m', 50.0)
        self.declare_parameter('limits.tcpa_min_s', 60.0)
        self.declare_parameter('limits.speed_max_mps', 2.0)
        self.declare_parameter('rules', ['COLREG-13','COLREG-14','COLREG-15','COLREG-16','COLREG-17'])
        self.declare_parameter('harbor_sop', ['NoGo: polygon:nogozone'])
        self.declare_parameter('constraints_pub_rate_hz', 0.5)

        # Environment params (optional)
        self.declare_parameter('env.wind_mps', 2.0)
        self.declare_parameter('env.current_mps', 0.3)
        self.declare_parameter('env.current_dir_deg', 90.0)

        # Target (optional)
        self.declare_parameter('target.enabled', True)
        self.declare_parameter('target.id', 't1')
        self.declare_parameter('target.type', 'barge')
        self.declare_parameter('target.pose.x', 180.0)
        self.declare_parameter('target.pose.y', 0.0)
        self.declare_parameter('target.pose.yaw_deg', 200.0)
        self.declare_parameter('target.vel.u', 1.0)
        self.declare_parameter('target.intent', 'drifting')

        # No-Go polygon (optional)
        # Example: [[120.0,40.0],[160.0,40.0],[160.0,80.0],[120.0,80.0]]
        self.declare_parameter('nogo.enabled', True)
        self.declare_parameter('nogo.id', 'nogozone')
        self.declare_parameter('nogo.points_flat', [120.0, 40.0, 160.0, 40.0, 160.0, 80.0, 120.0, 80.0])

        # Subscriptions
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.sub_odom = self.create_subscription(Odometry, odom_topic, self.on_odom, 10)

        # Publishers
        self.pub_state = self.create_publisher(WorldState, '/wm/state', 10)
        self.pub_constraints = self.create_publisher(Constraints, '/constraints', 10)

        # Timers
        self.timer_state = self.create_timer(1.0 / float(self.get_parameter('publish_rate_hz').value), self.tick_state)
        self.timer_constraints = self.create_timer(1.0 / float(self.get_parameter('constraints_pub_rate_hz').value), self.tick_constraints)

        # Internal state
        self.last_odom: Odometry = None

        self.get_logger().info(f'WorldModel started. Subscribing: {odom_topic}')

    # ---------------------------------------------------------------------
    # Callbacks
    # ---------------------------------------------------------------------
    def on_odom(self, msg: Odometry):
        self.last_odom = msg

    def tick_state(self):
        if self.last_odom is None:
            return
        m = WorldState()
        # stamp
        # m.stamp = Time(sec=self.get_clock().now().seconds_nanoseconds()[0],
        #                nanosec=self.get_clock().now().seconds_nanoseconds()[1])
        m.stamp = self.get_clock().now().to_msg()
        # ego
        od = self.last_odom
        q = od.pose.pose.orientation
        yaw_deg = quat_to_yaw_deg(q.x, q.y, q.z, q.w)
        ego = EgoState()
        ego.id = 'u1'
        ego.pose = Pose2D(x=od.pose.pose.position.x, y=od.pose.pose.position.y, yaw_deg=yaw_deg)
        ego.vel = Velocity2D(u=od.twist.twist.linear.x, v=od.twist.twist.linear.y, r=od.twist.twist.angular.z)
        # Assume OK unless overridden by other health monitors
        ego.health = Health(propulsion='ok', battery_soc=0.8, comm='ok')
        m.ego = ego

        # targets (optional, static for demo)
        if self.get_parameter('target.enabled').value:
            t = Target()
            t.id = self.get_parameter('target.id').value
            t.type = self.get_parameter('target.type').value
            t.pose = Pose2D(
                x=float(self.get_parameter('target.pose.x').value),
                y=float(self.get_parameter('target.pose.y').value),
                yaw_deg=float(self.get_parameter('target.pose.yaw_deg').value))
            t.vel = Velocity2D(u=float(self.get_parameter('target.vel.u').value), v=0.0, r=0.0)
            t.intent = self.get_parameter('target.intent').value
            m.targets = [t]
        else:
            m.targets = []

        # hazards (No-Go polygon)
        m.hazards = []
        if self.get_parameter('nogo.enabled').value:
            hz = Hazard()
            hz.id = self.get_parameter('nogo.id').value
            hz.type = 'polygon'
            hz.pose = Pose2D(x=0.0, y=0.0, yaw_deg=0.0)
            # pts_param: List[List[float]] = self.get_parameter('nogo.points').value
            # poly = Polygon(points=[Point2D(x=float(px), y=float(py)) for px, py in pts_param])
            # 변경 (2개씩 묶기)
            pts_flat: List[float] = self.get_parameter('nogo.points_flat').value
            pts: List[Point2D] = []
            if len(pts_flat) % 2 != 0:
                self.get_logger().warn('nogo.points_flat length must be even; last value will be ignored.')
            for i in range(0, len(pts_flat) - 1, 2):
                pts.append(Point2D(x=float(pts_flat[i]), y=float(pts_flat[i+1])))
            poly = Polygon(points=pts)
            hz.polygon = poly
            m.hazards.append(hz)

        # environment
        m.env = Env(
            wind_mps=float(self.get_parameter('env.wind_mps').value),
            current_mps=float(self.get_parameter('env.current_mps').value),
            current_dir_deg=float(self.get_parameter('env.current_dir_deg').value)
        )

        self.pub_state.publish(m)

    def tick_constraints(self):
        c = Constraints()
        c.rules = [str(s) for s in self.get_parameter('rules').value]
        lim = Limits(
            dcpa_min_m=float(self.get_parameter('limits.dcpa_min_m').value),
            tcpa_min_s=float(self.get_parameter('limits.tcpa_min_s').value),
            speed_max_mps=float(self.get_parameter('limits.speed_max_mps').value)
        )
        c.limits = lim
        c.harbor_sop = []  # fallback if param missing
        try:
            c.harbor_sop = [str(s) for s in self.get_parameter('harbor_sop').value]
        except Exception:
            c.harbor_sop = []
        c.vts_status = ''
        c.vts_msg = ''
        self.pub_constraints.publish(c)


def main():
    rclpy.init()
    node = WorldModelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
