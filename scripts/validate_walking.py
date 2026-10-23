#!/usr/bin/env python3
"""Exercise the real G1 policy on a released MuJoCo base."""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger

ROOT=Path(__file__).resolve().parents[1]

class Probe(Node):
    def __init__(self):
        super().__init__('g1_walking_probe')
        self.pose=None
        self.records=[]
        self.mode_pub=self.create_publisher(String,'/g1/controller_mode',10)
        self.cmd_pub=self.create_publisher(Twist,'/cmd_vel',10)
        self.create_subscription(PoseStamped,'/g1_sim/ground_truth',self.on_pose,10)
        self.reset_srv=self.create_client(Trigger,'/g1/reset')
        self.release_srv=self.create_client(Trigger,'/g1/release')
    def on_pose(self,msg):
        p=msg.pose.position;q=msg.pose.orientation
        self.pose=[float(p.x),float(p.y),float(p.z),float(q.w),float(q.x),float(q.y),float(q.z)]
    def run_for(self,seconds,mode,command=(0.,0.,0.)):
        m=String();m.data=mode
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            self.mode_pub.publish(m)
            t=Twist();t.linear.x,t.linear.y,t.angular.z=command
            self.cmd_pub.publish(t)
            rclpy.spin_once(self,timeout_sec=.05)
            if self.pose:self.records.append([time.time(),mode,*command,*self.pose])
    def call(self,client):
        if not client.wait_for_service(timeout_sec=5.):raise RuntimeError('Simulation service unavailable')
        future=client.call_async(Trigger.Request())
        end=time.monotonic()+5
        while not future.done() and time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.05)
        if not future.done() or not future.result().success:raise RuntimeError(str(future.result() if future.done() else 'timeout'))

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',default=str(ROOT/'sim/generated/walking_validation.json'))
    p.add_argument('--no-reset',action='store_true')
    args=p.parse_args();rclpy.init();node=Probe()
    try:
        node.run_for(.5,'passive')
        if not args.no_reset:node.call(node.reset_srv)
        node.run_for(3.,'stand')
        node.run_for(1.,'velocity')
        node.call(node.release_srv)
        node.run_for(3.,'velocity')
        node.run_for(7.,'velocity',(.15,0.,0.))
        node.run_for(3.,'velocity')
        node.run_for(5.,'velocity',(0.,0.,.25))
        node.run_for(3.,'velocity')
        Path(args.output).write_text(json.dumps(node.records))
        print('samples',len(node.records),'first',node.records[0] if node.records else None,'last',node.records[-1] if node.records else None)
    finally:
        node.run_for(.2,'velocity')
        node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
