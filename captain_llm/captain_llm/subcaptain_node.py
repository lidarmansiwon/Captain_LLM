#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from captain_interfaces.msg import CaptainOrder, CaptainResponse
from captain_interfaces.srv import ExecuteOrder

class SubCaptain(Node):
    def __init__(self):
        super().__init__('sub_captain')
        self.cli = self.create_client(ExecuteOrder, '/server/execute_order')
        self.create_subscription(CaptainOrder, '/captain/order', self.on_order, 10)
        self.resp_pub = self.create_publisher(CaptainResponse, '/subcaptain/response', 10)

    def on_order(self, msg: CaptainOrder):
        if not self.cli.service_is_ready():
            self.cli.wait_for_service(timeout_sec=5.0)
        req = ExecuteOrder.Request()
        req.order_id = msg.order_id
        req.detail = msg.detail
        future = self.cli.call_async(req)
        future.add_done_callback(lambda f: self._done(msg.order_id, f))

    def _done(self, order_id, future):
        resp_msg = CaptainResponse()
        resp_msg.order_id = order_id
        try:
            resp = future.result()
            resp_msg.ok = bool(resp.ok)
            resp_msg.msg = resp.msg
        except Exception as e:
            resp_msg.ok = False
            resp_msg.msg = f"service error: {e}"
        resp_msg.received_at = self.get_clock().now().to_msg()
        self.resp_pub.publish(resp_msg)
        self.get_logger().info(f"[SubCaptain] id={order_id} ok={resp_msg.ok} msg={resp_msg.msg}")

def main():
    rclpy.init()
    node = SubCaptain()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
