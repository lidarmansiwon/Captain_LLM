#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from captain_interfaces.srv import ExecuteOrder

class Server(Node):
    def __init__(self):
        super().__init__('server')
        # 콜백 이름 변경: self.handle_order
        self.srv = self.create_service(ExecuteOrder, '/server/execute_order', self.handle_order)

    def handle_order(self, req, resp):
        self.get_logger().info(f"[Server] order_id={req.order_id} detail={req.detail}")
        try:
            ans = input(f" -> Approve order {req.order_id}? (True/False): ").strip().lower()
            resp.ok = ans.startswith('t') or ans == '1' or ans == 'y'
            resp.msg = "user_input"
        except Exception as e:
            resp.ok = False
            resp.msg = f"input error: {e}"
        return resp

def main():
    rclpy.init()
    node = Server()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
