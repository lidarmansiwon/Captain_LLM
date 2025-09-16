#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from captain_interfaces.msg import WorldState, Constraints, HighLevelPlan, PlanDecision
from captain_interfaces.msg import Point2D


class RuleEngineNode(Node):
    def __init__(self):
        super().__init__('rule_engine')
        self.sub_w = self.create_subscription(WorldState, '/wm/state', self.on_world, 10)
        self.sub_c = self.create_subscription(Constraints, '/constraints', self.on_constraints, 10)
        self.sub_p = self.create_subscription(HighLevelPlan, '/plan/highlevel', self.on_plan, 10)
        self.pub_d = self.create_publisher(PlanDecision, '/plan/decision', 10)
        self.world=None; self.cons=None


    def on_world(self, msg: WorldState): self.world=msg
    def on_constraints(self, msg: Constraints): self.cons=msg


    def path_intersects_polygon(self, path, poly):
    # 간단 검사: waypoint가 폴리곤 내부면 true
        def wn(pt, poly):
            # winding number algorithm (요약)
            wn=0
            for i in range(len(poly.points)):
                a=poly.points[i]; b=poly.points[(i+1)%len(poly.points)]
                if a.y<=pt.y:
                    if b.y>pt.y and ((b.x-a.x)*(pt.y-a.y)-(pt.x-a.x)*(b.y-a.y))>0: wn+=1
                else:
                    if b.y<=pt.y and ((b.x-a.x)*(pt.y-a.y)-(pt.x-a.x)*(b.y-a.y))<0: wn-=1
            return wn!=0
        for wp in path:
            for hz in self.world.hazards:
                if hz.type=='polygon' and wn(wp, hz.polygon):
                    return True
        return False


    def on_plan(self, plan: HighLevelPlan):
        decision = PlanDecision(); decision.plan_id = plan.plan_id
        if self.world is None or self.cons is None:
            decision.status='REJECTED'; decision.reason='No world/constraints'; self.pub_d.publish(decision); return
        # 속도 제한 체크
        for a in plan.actions:
            if a.cmd=='set_speed' and a.value_mps > self.cons.limits.speed_max_mps + 1e-6:
                decision.status='REJECTED'; decision.reason='Speed limit'; self.pub_d.publish(decision); return
        # No-Go 체크
        if self.path_intersects_polygon(plan.waypoints, None):
            decision.status='REJECTED'; decision.reason='No-Go violation'; self.pub_d.publish(decision); return
        # DCPA/TCPA 근사(첫 웨이포인트 기준)
        ego = self.world.ego
        p_o = (ego.pose.x, ego.pose.y)
        yaw = math.radians(ego.pose.yaw_deg)
        v_o = (ego.vel.u*math.cos(yaw), ego.vel.u*math.sin(yaw))
        if len(plan.waypoints)>0:
            p_first = (plan.waypoints[0].x, plan.waypoints[0].y)
        else:
            p_first = p_o
        for tgt in self.world.targets:
            p_t = (tgt.pose.x, tgt.pose.y)
            v_t = (tgt.vel.u*math.cos(math.radians(tgt.pose.yaw_deg)), tgt.vel.u*math.sin(math.radians(tgt.pose.yaw_deg)))
            dcpa, tcpa = self.dcpa_tcpa(p_first, v_o, p_t, v_t)
            if dcpa < self.cons.limits.dcpa_min_m or tcpa < self.cons.limits.tcpa_min_s:
                decision.status='REJECTED'; decision.reason=f'CPA {dcpa:.1f}/{tcpa:.1f}'; self.pub_d.publish(decision); return
        decision.status='APPROVED'; decision.reason='ok'; self.pub_d.publish(decision)


    def dcpa_tcpa(self, p_o, v_o, p_t, v_t):
        rx, ry = p_t[0]-p_o[0], p_t[1]-p_o[1]
        rvx, rvy = v_t[0]-v_o[0], v_t[1]-v_o[1]
        rv2 = rvx*rvx + rvy*rvy
        if rv2 < 1e-9: return (math.hypot(rx,ry), 0.0)
        tcpa = - (rx*rvx + ry*rvy) / rv2
        dcpa = math.hypot(rx+rvx*tcpa, ry+rvy*tcpa)
        return (dcpa, tcpa)




def main():
    rclpy.init(); rclpy.spin(RuleEngineNode()); rclpy.shutdown()