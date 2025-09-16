#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from captain_interfaces.msg import WorldState, Constraints, PlannerRequest, Event, Polygon, Point2D

class FSMNode(Node):
    def __init__(self):
        super().__init__('mission_fsm')
        self.state='SELF_CHECK'
        self.sub_w = self.create_subscription(WorldState, '/wm/state', self.on_world, 10)
        self.sub_c = self.create_subscription(Constraints, '/constraints', self.on_constraints, 10)
        self.pub_req = self.create_publisher(PlannerRequest, '/planner/request', 10)
        self.pub_evt = self.create_publisher(Event, '/events/state', 10)
        self.timer = self.create_timer(1.0, self.tick)
        self.world=None; self.cons=None

    def on_world(self, m: WorldState): self.world=m
    def on_constraints(self, m: Constraints): self.cons=m

    def tick(self):
        if self.state=='SELF_CHECK' and self.world and self._ok(self.world):
            self._to('STANDBY')
        elif self.state=='STANDBY' and self.cons:
            req=PlannerRequest(); req.mission_type='intercept_and_escort'; req.policy_style='conservative'; req.update_period_s=5.0
            req.goal_zone=Polygon(points=[Point2D(x=30.0,y=-30.0),Point2D(x=60.0,y=-30.0),Point2D(x=60.0,y=30.0),Point2D(x=30.0,y=30.0)])
            self.pub_req.publish(req); self._to('MISSION_ACTIVE')

    def _to(self, s):
        self.state=s; e=Event(); e.event='STATE_CHANGE'; e.detail=s; self.pub_evt.publish(e)

    def _ok(self, w: WorldState):
        h=w.ego.health; return (h.propulsion=='ok') and (h.battery_soc>0.2) and (h.comm in ['ok','warn'])


def main():
    rclpy.init(); rclpy.spin(FSMNode()); rclpy.shutdown()
