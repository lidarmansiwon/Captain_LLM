#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from captain_interfaces.msg import WorldState, Constraints, PlannerRequest, HighLevelPlan, Point2D, Action


class CaptainNode(Node):
    def __init__(self):
        super().__init__('captain_llm')
        self.sub_w = self.create_subscription(WorldState, '/wm/state', self.on_world, 10)
        self.sub_c = self.create_subscription(Constraints, '/constraints', self.on_constraints, 10)
        self.sub_r = self.create_subscription(PlannerRequest, '/planner/request', self.on_request, 10)
        self.pub_p = self.create_publisher(HighLevelPlan, '/plan/highlevel', 10)
        self.world = None; self.cons = None


    def on_world(self, msg: WorldState):
        self.world = msg    


    def on_constraints(self, msg: Constraints):
        self.cons = msg


    def on_request(self, req: PlannerRequest):
        if self.world is None or self.cons is None:
            return
        ## TODO: 상태 요약 + LLM 호출 → 여기선 데모 플랜 생성
        plan = HighLevelPlan()
        plan.plan_id = 'p_demo'
        plan.waypoints = [Point2D(x=90.0,y=0.0), Point2D(x=70.0,y=0.0), Point2D(x=55.0,y=0.0)]
        a = Action(); a.cmd='set_speed'; a.value_mps=1.6; a.condition=''
        plan.actions = [a]
        plan.rationale = 'demo: right-block approach under COLREG'
        plan.safety_checks = ['DCPA>50','TCPA>60']
        plan.fallback = 'hold at [20,-10]'
        self.pub_p.publish(plan)

def main():
    rclpy.init(); rclpy.spin(CaptainNode()); rclpy.shutdown()