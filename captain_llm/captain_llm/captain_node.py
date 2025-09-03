#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from captain_interfaces.msg import ShipStatus, CaptainOrder, CaptainResponse

class Captain(Node):
    def __init__(self):
        super().__init__('captain')

        # 상태
        self.started = False
        self.status = ShipStatus()
        self.status.navigation = False
        self.status.thruster = False
        self.status.operation = False
        self.status.note = "Waiting for Mission Start"

        self.nav_health = False
        self.create_subscription(Bool, '/nav/health', self.on_nav_health, 10)

        # 주문 큐 & 진행 중
        self.queue = []           # [(order_id, detail)]
        self.in_flight = None

        # 인터페이스
        self.voice_sub  = self.create_subscription(String, '/voice_cmd', self.on_voice_cmd, 10)
        self.order_pub  = self.create_publisher(CaptainOrder, '/captain/order', 10)
        self.resp_sub   = self.create_subscription(CaptainResponse, '/subcaptain/response', self.on_response, 10)
        self.ship_pub   = self.create_publisher(ShipStatus, '/ship/status', 10)
        self.log_pub    = self.create_publisher(String, '/captain/status', 10)

        # 상태 주기 방송
        self.create_timer(0.5, self.publish_status)
        self._log("CAPTAIN_READY. Say 'Mission Start' to begin.")

    def on_nav_health(self, msg: Bool):
        prev_nav = self.status.navigation
        self.nav_health = bool(msg.data)
        self.status.navigation = self.nav_health

        # 운항 중인데 네비가 끊기면 즉시 운항 false
        if not self.nav_health and self.status.operation:
            self.status.operation = False
            self.status.note = "Navigation STALE → operation stopped."
            self._log(self.status.note)
        elif self.status.navigation != prev_nav:
            self.status.note = "Navigation ALIVE" if self.status.navigation else "Navigation STALE"
            self._log(self.status.note)

    # ---------- 입력 처리 ----------
    def on_voice_cmd(self, msg: String):
        # rosgpt.py → {"text":"...", "json":"{... standard ...}"}
        try:
            payload = json.loads(msg.data)
            final_json = json.loads(payload.get("json","{}"))
        except Exception:
            self._log("invalid payload; ignored")
            return

        mission = final_json.get("mission", "none")
        orders  = final_json.get("orders", [])
        note    = final_json.get("note", "")

        if not self.started:
            if mission == "start":
                self.started = True
                self.status.navigation = False
                self.status.thruster = False
                self.status.operation = False
                self.status.note = "Mission started. (nav,thruster,operation = False)"
                self._log("MISSION_START: broadcasting schedule & status.")
                self._broadcast_schedule()
            else:
                # mission != start 인 모든 입력은 무시
                self._log("IGNORED: Mission not started.")
            return

        # 이미 시작한 이후: orders가 있으면 순차 enqueue
        # if orders:
        #     for od in orders:
        #         oid = int(od.get("order_id", -1))
        #         det = str(od.get("detail", ""))
        #         if oid > 0:
        #             self._enqueue(oid, det)
        #     return
        if orders:
            for od in orders:
                oid = int(od.get("order_id", -1))
                det = str(od.get("detail", ""))
                if oid <= 0:
                    continue
                if oid == 5:
                    # 출항은 선행조건 필수 검사
                    self._depart_intent()
                else:
                    self._enqueue(oid, det)
            return


        # orders가 없으면 note만 갱신하거나 무시
        if note:
            self.status.note = note
            self._log(f"NOTE: {note}")

    # ---------- 의도 처리 ----------
    def _handle_json_intent(self, j: dict) -> bool:
        action = str(j.get('action', '')).lower()
        if action in ('check_navigation', 'nav_check'):
            self._enqueue(4, "navigation check (LLM)")
        elif action in ('check_thruster', 'propulsor_check'):
            self._enqueue(3, "thruster check (LLM)")
        elif action in ('check_ship_status', 'inspection'):
            self._enqueue(3, "thruster check (LLM seq)")
            self._enqueue(4, "navigation check (LLM seq)")
        elif action in ('depart', 'sail'):
            self._depart_intent()
        else:
            self._log(f"Unknown LLM action: {action}")
            return False
        return True

    def _handle_text_intent(self, text: str):
        t = text.lower()
        if any(k in t for k in ['선박 상태 검사', '상태 검사', 'check ship status', 'inspection']):
            self._enqueue(3, "thruster check (text seq)")
            self._enqueue(4, "navigation check (text seq)")
        elif any(k in t for k in ['항법', 'navigation']):
            self._enqueue(4, "navigation check (text)")
        elif any(k in t for k in ['추진기', 'thruster', 'propulsor']):
            self._enqueue(3, "thruster check (text)")
        elif any(k in t for k in ['출항', 'depart', 'sail']):
            self._depart_intent()
        elif any(k in t for k in ['schedule', '스케줄', 'plan']):
            self._broadcast_schedule()
        else:
            self._log(f"No intent matched: {text[:80]}")

    # def _depart_intent(self):
    #     missing = []
    #     if not self.status.navigation:
    #         missing.append("navigation")
    #     if not self.status.thruster:
    #         missing.append("thruster")
    #     if missing:
    #         self.status.note = f"Cannot depart; missing checks: {', '.join(missing)}"
    #         self._log(self.status.note)
    #         return
    #     self._enqueue(5, "departure")
    def _depart_intent(self):
        missing = []
        if not self.status.navigation:
            missing.append(("navigation", 4))
        if not self.status.thruster:
            missing.append(("thruster", 3))

        if missing:
            names = [m[0] for m in missing]
            self.status.note = f"Cannot depart; missing checks: {', '.join(names)}"
            self._log(self.status.note)

            # 자동 검사 지시 (부족한 것들만 순서대로 enqueue)
            for name, oid in missing:
                # 이미 진행 중이거나 큐에 있는지 간단히 중복 방지(선택)
                if (oid, "auto") not in self.queue:
                    self._enqueue(oid, f"{name} check (auto)")
            return

        # 선행조건 충족 시에만 출항명령
        self._enqueue(5, "departure")

    # ---------- 큐 & 발행 ----------
    # def _enqueue(self, order_id: int, detail: str):
    #     self.queue.append((order_id, detail))
    #     self._log(f"ENQUEUE order {order_id}: {detail}")
    #     self._try_dispatch()

    def _enqueue(self, order_id: int, detail: str):
        # 재검사 들어갈 때 미리 상태를 False로 떨어뜨림
        if order_id == 3:   # Thruster check
            if self.status.thruster:  # 이미 True였다면 재검중으로 False
                self.status.thruster = False
                self.status.note = "Rechecking thruster..."
        elif order_id == 4: # Navigation check
            if self.status.navigation:
                self.status.navigation = False
                self.status.note = "Rechecking navigation..."

        self.queue.append((order_id, detail))
        self._log(f"ENQUEUE order {order_id}: {detail}")
        self._try_dispatch()

    def _try_dispatch(self):
        if self.in_flight is not None:
            return
        if not self.queue:
            return
        order_id, detail = self.queue.pop(0)
        self.in_flight = order_id
        msg = CaptainOrder()
        msg.order_id = order_id
        msg.detail = detail
        msg.issued_at = self.get_clock().now().to_msg()
        self.order_pub.publish(msg)
        self._log(f"ORDER_SENT id={order_id} detail={detail}")

    # ---------- 결과 처리 ----------
    # def on_response(self, msg: CaptainResponse):
    #     self._log(f"RESULT id={msg.order_id} ok={msg.ok} msg={msg.msg}")
    #     if msg.order_id == 3 and msg.ok:
    #         self.status.thruster = True
    #         self.status.note = "Thruster check OK."
    #     elif msg.order_id == 4 and msg.ok:
    #         self.status.navigation = True
    #         self.status.note = "Navigation check OK."
    #     elif msg.order_id == 5 and msg.ok:
    #         self.status.operation = True
    #         self.status.note = "Departure executed."

    #     self.in_flight = None
    #     self._try_dispatch()

    def on_response(self, msg: CaptainResponse):
        self._log(f"RESULT id={msg.order_id} ok={msg.ok} msg={msg.msg}")

        if msg.order_id == 3:
            self.status.thruster = bool(msg.ok)
            self.status.note = "Thruster check OK." if msg.ok else "Thruster check FAILED."
        elif msg.order_id == 4:
            self.status.navigation = bool(msg.ok)
            self.status.note = "Navigation check OK." if msg.ok else "Navigation check FAILED."
        elif msg.order_id == 5:
            self.status.operation = bool(msg.ok)
            self.status.note = "Departure executed." if msg.ok else "Departure failed."

        self.in_flight = None
        self._try_dispatch()

    # ---------- 보조 ----------
    def publish_status(self):
        self.ship_pub.publish(self.status)

    def _broadcast_schedule(self):
        self._log("SCHEDULE: 1) check thruster  2) check navigation  3) depart")

    def _has_mission_start(self, text: str, json_str: str) -> bool:
        s = (text + ' ' + json_str).lower()
        return any(k in s for k in ['mission start', 'start mission', '임무 시작', '미션 시작'])

    def _log(self, s: str):
        self.get_logger().info(f"[Captain] {s}")
        self.log_pub.publish(String(data=s))

def main():
    rclpy.init()
    node = Captain()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
