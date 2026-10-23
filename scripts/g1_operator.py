#!/usr/bin/env python3
"""State-checked operator stages for the MuJoCo G1 navigation demo."""
import argparse
import json
import math
import os
import time
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped,Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import String,Bool,Float32
from std_srvs.srv import Trigger
from action_msgs.srv import CancelGoal

ROOT=Path(__file__).resolve().parents[1]
DOMAIN=os.environ.get('ROS_DOMAIN_ID','72')
STATUS_FILE=ROOT/f'.deps/g1-controller/status-{DOMAIN}.txt'

class Operator(Node):
 def __init__(self):
  super().__init__('g1_operator')
  self.mode_pub=self.create_publisher(String,'/g1/controller_mode',10)
  self.enable_pub=self.create_publisher(Bool,'/g1/nav_enable',10)
  self.cmd_pub=self.create_publisher(Twist,'/cmd_vel',10)
  self.release=self.create_client(Trigger,'/g1/release')
  self.cancel=self.create_client(CancelGoal,'/navigate_to_pose/_action/cancel_goal')
  self.pose=None;self.joints=None;self.released=False;self.release_time=0.
  self.confidence=0.;self.conf_time=0.;self.odom_time=0.;self.nav_ready=False
  self.create_subscription(PoseStamped,'/g1_sim/ground_truth',lambda m:setattr(self,'pose',m),10)
  self.create_subscription(JointState,'/joint_states',lambda m:setattr(self,'joints',m),10)
  self.create_subscription(Bool,'/g1/released',self.on_released,10)
  self.create_subscription(Float32,'/localization_3d_confidence',self.on_conf,10)
  self.create_subscription(Odometry,'/odom',lambda m:setattr(self,'odom_time',time.monotonic()),10)
  self.create_subscription(Bool,'/g1/nav_ready',lambda m:setattr(self,'nav_ready',m.data),10)
 def on_released(self,m):self.released=m.data;self.release_time=time.monotonic()
 def on_conf(self,m):self.confidence=m.data;self.conf_time=time.monotonic()
 def until(self,condition,seconds):
  end=time.monotonic()+seconds
  while time.monotonic()<end:
   rclpy.spin_once(self,timeout_sec=.05)
   if condition():return True
  return False
 def mode(self):
  try:
   stamp,mode=STATUS_FILE.read_text().split()[:2]
   return int(mode) if time.time()-float(stamp)<.5 else -1
  except (OSError,IndexError,ValueError):return -1
 def publish_mode(self,s):
  m=String();m.data=s;self.mode_pub.publish(m)
 def publish_enable(self,value):
  m=Bool();m.data=value;self.enable_pub.publish(m)
 def status(self):
  p=self.pose.pose.position if self.pose else None
  return {'controller_mode':self.mode(),'released':self.released,
          'pose_xyz':[p.x,p.y,p.z] if p else None,
          'confidence':self.confidence if time.monotonic()-self.conf_time<1 else None,
          'odom_fresh':time.monotonic()-self.odom_time<.4,'nav_ready':self.nav_ready}
 def prepare(self):
  assert self.until(lambda:self.pose and self.joints and self.mode()>0,8),'simulator or controller unavailable'
  if self.released:
   assert self.mode()==3,'base released without walking controller'
   return
  self.publish_mode('stand')
  assert self.until(lambda:self.mode()==2,5),'FixStand transition failed'
  target=yaml.safe_load((ROOT/'.deps/g1-controller/config.yaml').read_text())['FSM']['FixStand']['qs'][-1]
  def settled():
   if not self.joints or len(self.joints.position)<len(target):return False
   error=math.sqrt(sum((a-b)**2 for a,b in zip(self.joints.position,target))/len(target))
   return error<.12
  assert self.until(settled,6),'stand joints did not settle'
  self.publish_mode('velocity')
  assert self.until(lambda:self.mode()==3,5),'walking policy transition failed'
  assert self.until(lambda:self.pose and .75<self.pose.pose.position.z<.83,2),'supported pose abnormal'
  assert self.release.wait_for_service(timeout_sec=3),'release service unavailable'
  f=self.release.call_async(Trigger.Request())
  assert self.until(lambda:f.done(),3) and f.result().success,'base release refused'
  assert self.until(lambda:self.released,2),'release state not published'
 def enable_nav(self):
  assert self.until(lambda:self.released and self.mode()==3 and self.confidence>=.6 and
                    time.monotonic()-self.conf_time<.8 and
                    time.monotonic()-self.odom_time<.4,5), \
         'controller, release, odometry or localization not ready'
  self.publish_enable(True)
  assert self.until(lambda:self.nav_ready,3),'navigation gate refused enable'
 def stop(self):
  self.publish_enable(False);self.cmd_pub.publish(Twist())
  if self.cancel.wait_for_service(timeout_sec=.5):
   f=self.cancel.call_async(CancelGoal.Request());self.until(lambda:f.done(),3)
  self.until(lambda:False,.5)

def main():
 p=argparse.ArgumentParser();p.add_argument('action',choices=['prepare','enable-nav','stop','status']);args=p.parse_args()
 rclpy.init();node=Operator()
 try:
  if args.action=='prepare':node.prepare()
  elif args.action=='enable-nav':node.enable_nav()
  elif args.action=='stop':node.stop()
  else:
   node.until(lambda:node.pose is not None and node.joints is not None and
              node.conf_time>0 and node.odom_time>0,5)
  print(json.dumps(node.status()))
 finally:node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
