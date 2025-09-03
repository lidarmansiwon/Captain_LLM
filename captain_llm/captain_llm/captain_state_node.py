#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.duration import Duration
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from captain_llm.schedule_store import ScheduleStore
from captain_llm.srv import CaptainOrder  # srv 패키지명에 맞춰 import

class CaptainStateNode(Node):
    def __init__(self):
        super().__init__('captain_state_node')

        # 파라미터
        self.declare_parameter('schedule_file', 'config/schedule.yaml')
        self.declare_parameter('order_service', '/captain/order')
        self.schedule_file = self.get_parameter('schedule_file').value
        self.order_service_name = self.get_parameter('order_service').value

        # 퍼블리셔
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.status_pub = self.create_publisher(String, '/captain/status', qos)
        self.wp_pub = self.create_publisher(Path,   '/captain/waypoints', qos)

        # 서비스 클라이언트
        self.order_cli = self.create_client(CaptainOrder, self.order_service_name)
        if not self.order_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(f"Service {self.order_service_name} not available now. Will retry on demand.")

        # 스케줄 로드
        self.store = ScheduleStore(self.schedule_file)
        self.schedule = self.store.get_schedule()
        self.waypoints = self.store.get_waypoints()

        self._publish_waypoints(self.waypoints)
        self.state_idx = 0
        self.inspection_ok = False  # 점검 결과 캐시

        # 상태머신 타이머
        self.timer = self.create_timer(0.5, self._tick)  # 2Hz

        self._broadcast(f"CAPTAIN_READY. Schedule size={len(self.schedule)}")

    # 상태머신 한 스텝
    def _tick(self):
        if self.state_idx >= len(self.schedule):
            return  # 종료

        cur = self.schedule[self.state_idx]
        task = cur.get('task', '').upper()

        if task == 'WAIT':
            self._broadcast(f"WAIT at {cur.get('t')}")
            self._advance()

        elif task == 'INSPECTION':
            self._broadcast("INSPECTION_START")
            steps = cur.get('steps', [])
            self._run_steps_sync(steps)  # 각 step은 CaptainOrder 서비스 True/False
            self._advance()

        elif task == 'INSPECTION_RESULT':
            # 간단 규칙: 앞 단계들 OK면 True
            rule = cur.get('rule', 'DEPARTURE_ELIGIBLE')
            ok = self._compute_rule(rule)
            self.inspection_ok = ok
            self._broadcast(f"INSPECTION_RESULT={ok}")
            self._advance()

        elif task == 'WAIT_UNTIL_DEPARTURE':
            depart_at = cur.get('depart_at')
            if depart_at:
                # 여기서는 간단히 바로 통과(필요하면 rclpy.clock로 현재시간 비교)
                self._broadcast(f"WAIT_UNTIL_DEPARTURE until {depart_at} (skipped demo)")
            else:
                self._broadcast("WAIT_UNTIL_DEPARTURE(no time given)")

            self._advance()

        elif task == 'DEPART':
            if not self.inspection_ok:
                self._broadcast("DEPART_ABORTED (inspection not ok)")
                self._advance()
                return
            step = cur.get('step', {})
            ok = self._call_order_blocking(step.get('order_id', 5), step.get('detail', 'departure'))
            if ok:
                self._broadcast("DEPARTED; STATUS=SAILING")
            else:
                self._broadcast("DEPART_FAILED")
            self._advance()

        else:
            self._broadcast(f"UNKNOWN_TASK:{task}")
            self._advance()

    def _advance(self):
        self.state_idx += 1

    def _broadcast(self, text: str):
        msg = String(); msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(f"[CAPTAIN] {text}")

    def _run_steps_sync(self, steps):
        for s in steps:
            oid = int(s.get('order_id', -1))
            detail = str(s.get('detail', ''))
            ok = self._call_order_blocking(oid, detail)
            self._broadcast(f"STEP order_id={oid} ok={ok}")
            if not ok:
                break

    def _call_order_blocking(self, order_id: int, detail: str) -> bool:
        if not self.order_cli.service_is_ready():
            if not self.order_cli.wait_for_service(timeout_sec=3.0):
                self.get_logger().error(f"Service {self.order_service_name} not available")
                return False

        req = CaptainOrder.Request()
        req.order_id = order_id
        req.detail = detail

        future: Future = self.order_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        if not future.done():
            self.get_logger().error("CaptainOrder timeout")
            return False
        resp = future.result()
        if resp is None:
            self.get_logger().error("CaptainOrder no response")
            return False
        self.get_logger().info(f"Order({order_id}) -> ok={resp.ok}, msg={resp.msg}")
        return bool(resp.ok)

    def _compute_rule(self, rule: str) -> bool:
        # 데모: 직전 INSPECTION 단계의 모든 step이 True였는지를 기반으로 판단하고 싶다면,
        # 간단히 self.inspection_ok 를 계속 True로 유지하는 방식 대신,
        # _run_steps_sync에서 실패 시 플래그를 두는 식으로 확장 가능.
        # 여기서는 데모로 True로 둔다.
        return True

    def _publish_waypoints(self, wps):
        # (옵션) nav_msgs/Path로 퍼블리시 — lat/lon을 지도 좌표로 변환하지 않고 그냥 header만 채우는 데모 버전
        path = Path()
        path.header.frame_id = "map"
        for i, (lat, lon) in enumerate(wps):
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x = float(lon)  # 실제론 지도로 변환 필요(예: WGS84→ENU)
            ps.pose.position.y = float(lat)
            path.poses.append(ps)
        self.wp_pub.publish(path)

def main():
    rclpy.init()
    node = CaptainStateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
