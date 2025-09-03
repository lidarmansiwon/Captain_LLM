#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from captain_interfaces.msg import ShipStatus   # captain_interfaces 패키지에 정의된 ShipStatus.msg


class ThrusterCommander(Node):
    def __init__(self):
        super().__init__('thruster_commander')

        # Publishers for thrusters
        self.left_pub = self.create_publisher(Float64, '/agent0/thrusters/left/thrust', 10)
        self.right_pub = self.create_publisher(Float64, '/agent0/thrusters/right/thrust', 10)

        # Subscribe to ShipStatus
        self.sub = self.create_subscription(
            ShipStatus,
            '/ship/status',
            self.on_status,
            10
        )

        # 내부 상태 저장
        self.nav_ok = False
        self.thruster_ok = False
        self.operation_ok = False

        # 20Hz 타이머 (0.05초마다 발행)
        self.timer = self.create_timer(0.05, self.publish_thrusters)

        self.get_logger().info("ThrusterCommander started (publishing at 20Hz)")

    def on_status(self, msg: ShipStatus):
        # 최신 상태 업데이트
        self.nav_ok = msg.navigation
        self.thruster_ok = msg.thruster
        self.operation_ok = msg.operation

    def publish_thrusters(self):
        thrust_msg = Float64()

        if self.nav_ok and self.thruster_ok and self.operation_ok:
            thrust_msg.data = 180.0
        else:
            thrust_msg.data = 0.0

        self.left_pub.publish(thrust_msg)
        self.right_pub.publish(thrust_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
