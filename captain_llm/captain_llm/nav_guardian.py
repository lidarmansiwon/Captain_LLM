#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import Bool
from nav_msgs.msg import Odometry

import requests  # LLM 알림 옵션용

class NavGuardian(Node):
    def __init__(self):
        super().__init__('nav_guardian')

        # 파라미터
        self.declare_parameter('odom_topic', '/agent_0/odom')
        self.declare_parameter('stale_timeout', 1.0)       # sec
        self.declare_parameter('check_hz', 10.0)           # Hz
        self.declare_parameter('enable_llm_notify', True)  # STALE 전환 시 한 번 알림
        self.declare_parameter('llm_server_url', 'http://localhost:5000/rosgpt')
        self.declare_parameter('client_id', 'nav_guardian')
        self.declare_parameter('notify_cooldown_sec', 30.0)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.stale_timeout = float(self.get_parameter('stale_timeout').value)
        self.check_period = 1.0 / float(self.get_parameter('check_hz').value)
        self.enable_llm = bool(self.get_parameter('enable_llm_notify').value)
        self.server_url = self.get_parameter('llm_server_url').value
        self.client_id = self.get_parameter('client_id').value
        self.cooldown = float(self.get_parameter('notify_cooldown_sec').value)

        # 상태
        self.last_recv_time: Time | None = None
        self.alive = False
        self.last_notify_ts = 0.0

        # ROS I/O
        self.sub = self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.pub = self.create_publisher(Bool, '/nav/health', 10)
        self.timer = self.create_timer(self.check_period, self.tick)

        self.get_logger().info(
            f"[NavGuardian] watching {self.odom_topic}, stale_timeout={self.stale_timeout}s, "
            f"LLM_notify={'on' if self.enable_llm else 'off'}"
        )

    def on_odom(self, msg: Odometry):
        # 최신 수신 시각 갱신
        self.last_recv_time = self.get_clock().now()

    def tick(self):
        now = self.get_clock().now()
        alive_now = False
        if self.last_recv_time is not None:
            dt = (now - self.last_recv_time).nanoseconds / 1e9
            alive_now = (dt <= self.stale_timeout)

        # 상태 변동 로그 및 LLM 알림
        if alive_now != self.alive:
            self.get_logger().info(f"[NavGuardian] navigation {'ALIVE' if alive_now else 'STALE'}")
            if (not alive_now) and self.enable_llm:
                self._notify_llm_once("Navigation data lost. Please re-check the navigation system.")

        self.alive = alive_now

        # 브로드캐스트
        m = Bool(); m.data = self.alive
        self.pub.publish(m)

    # LLM에 “문제 발생”을 한 번만 알림 (쿨다운)
    def _notify_llm_once(self, text_command: str):
        now_ts = time.time()
        if now_ts - self.last_notify_ts < self.cooldown:
            return
        self.last_notify_ts = now_ts

        try:
            data = {'text_command': text_command, 'client_id': self.client_id}
            # 타임아웃 짧게
            r = requests.post(self.server_url, data=data, timeout=3.0)
            if r.status_code == 200:
                self.get_logger().info("[NavGuardian] LLM notified about nav issue.")
            else:
                self.get_logger().warn(f"[NavGuardian] LLM notify HTTP {r.status_code}")
        except Exception as e:
            self.get_logger().warn(f"[NavGuardian] LLM notify failed: {e}")

def main():
    rclpy.init()
    node = NavGuardian()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
