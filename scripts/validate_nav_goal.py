#!/usr/bin/env python3
"""End-to-end G1 walking, LIO localization and one Nav2 goal."""
import json
import argparse
import math
import os
import time
from pathlib import Path
import rclpy
import rclpy.time
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path as RosPath
from std_msgs.msg import String, Bool, Float32
from std_srvs.srv import Trigger, SetBool
from tf2_ros import Buffer, TransformListener

ROOT=Path(__file__).resolve().parents[1]
DOMAIN=os.environ.get('ROS_DOMAIN_ID','72')
STATUS_FILE=ROOT/f'.deps/g1-controller/status-{DOMAIN}.txt'
class Probe(Node):
 def __init__(self):
  super().__init__('g1_nav_goal_probe',parameter_overrides=[
   Parameter('use_sim_time',Parameter.Type.BOOL,True)])
  self.mode=self.create_publisher(String,'/g1/controller_mode',10)
  self.enable=self.create_publisher(Bool,'/g1/nav_enable',10)
  self.manual=self.create_publisher(Twist,'/cmd_vel',10)
  self.tf_buffer=Buffer()
  self.tf_listener=TransformListener(self.tf_buffer,self)
  self.action=ActionClient(self,NavigateToPose,'navigate_to_pose')
  self.release=self.create_client(Trigger,'/g1/release')
  self.obstacle=self.create_client(SetBool,'/g1/obstacle')
  self.truth=None;self.odom=None;self.conf=0.;self.ready=False;self.commands=[];self.trace=[]
  self.obstacle_active=False;self.obstacle_xy=None;self.min_obstacle_distance=None
  self.plan_updates=0
  self.create_subscription(PoseStamped,'/g1_sim/ground_truth',self.on_truth,10)
  self.create_subscription(Odometry,'/odom',lambda m:setattr(self,'odom',m),10)
  self.create_subscription(Float32,'/localization_3d_confidence',lambda m:setattr(self,'conf',m.data),10)
  self.create_subscription(Bool,'/g1/nav_ready',lambda m:setattr(self,'ready',m.data),10)
  self.create_subscription(Twist,'/cmd_vel_nav',lambda m:self.commands.append([time.time(),m.linear.x,m.angular.z]),10)
  self.create_subscription(Bool,'/g1/obstacle_active',lambda m:setattr(self,'obstacle_active',m.data),10)
  self.create_subscription(PoseStamped,'/g1/obstacle_pose',self.on_obstacle,10)
  self.create_subscription(RosPath,'/plan',self.on_plan,10)
 def on_obstacle(self,m):
  self.obstacle_xy=[float(m.pose.position.x),float(m.pose.position.y)]
 def on_plan(self,m):
  self.plan_updates+=1
 def on_truth(self,m):
  self.truth=m
  global_xy=[None,None]
  try:
   tf=self.tf_buffer.lookup_transform('map','base_footprint',rclpy.time.Time())
   global_xy=[tf.transform.translation.x,tf.transform.translation.y]
  except Exception:
   pass
  if self.odom:self.trace.append([time.time(),m.pose.position.x,m.pose.position.y,m.pose.position.z,
                                  self.odom.pose.pose.position.x,self.odom.pose.pose.position.y,self.conf,self.ready,
                                  global_xy[0],global_xy[1]])
  if self.obstacle_active and self.obstacle_xy:
   distance=math.hypot(m.pose.position.x-self.obstacle_xy[0],
                       m.pose.position.y-self.obstacle_xy[1])
   self.min_obstacle_distance=(distance if self.min_obstacle_distance is None
                               else min(self.min_obstacle_distance,distance))
 def spin_until(self,pred,timeout):
  end=time.monotonic()+timeout
  while time.monotonic()<end:
   rclpy.spin_once(self,timeout_sec=.05)
   if pred():return True
  return False
 def set_mode(self,name):
  m=String();m.data=name;self.mode.publish(m)
 def set_nav(self,enabled):
  m=Bool();m.data=enabled;self.enable.publish(m)
 def set_mode_until(self,name,expected,timeout):
  end=time.monotonic()+timeout
  while time.monotonic()<end:
   self.set_mode(name)
   rclpy.spin_once(self,timeout_sec=.05)
   if self.controller_mode()==expected:return True
  return False
 def set_nav_until(self,enabled,predicate,timeout):
  end=time.monotonic()+timeout
  while time.monotonic()<end:
   self.set_nav(enabled)
   rclpy.spin_once(self,timeout_sec=.05)
   if predicate():return True
  return False
 def set_obstacle(self,enabled):
  if not self.obstacle.wait_for_service(timeout_sec=3):
   return False
  request=SetBool.Request();request.data=enabled
  future=self.obstacle.call_async(request)
  if not self.spin_until(lambda:future.done(),3):return False
  return bool(future.result().success)
 def controller_mode(self):
  try:return int(STATUS_FILE.read_text().split()[1])
  except (OSError,IndexError,ValueError):return -1

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--x',type=float,default=1.5)
 parser.add_argument('--y',type=float,default=0.);parser.add_argument('--timeout',type=float,default=45.)
 parser.add_argument('--yaw',type=float,default=0.)
 parser.add_argument('--cancel-after',type=float,default=None)
 parser.add_argument('--obstacle-after',type=float,default=None)
 parser.add_argument('--obstacle-clear-after',type=float,default=None)
 parser.add_argument('--output',default=str(ROOT/'sim/generated/nav_goal_validation.json'))
 args=parser.parse_args();rclpy.init();n=Probe();result={'goal':[args.x,args.y,args.yaw]}
 try:
  assert n.spin_until(lambda:n.truth and n.odom and n.conf>=.6,10),'localization unavailable'
  if n.controller_mode()!=3:
   assert n.set_mode_until('stand',2,5),'stand transition unavailable'
   n.spin_until(lambda:False,3)
   assert n.set_mode_until('velocity',3,5),'walking policy unavailable'
   n.spin_until(lambda:False,1)
  assert n.release.wait_for_service(timeout_sec=3),'release service unavailable'
  f=n.release.call_async(Trigger.Request())
  assert n.spin_until(lambda:f.done(),3) and f.result().success,'release failed'
  assert n.set_nav_until(True,lambda:n.ready,5),'navigation gate not ready'
  assert n.action.wait_for_server(timeout_sec=5),'Nav2 action unavailable'
  g=NavigateToPose.Goal();g.pose.header.frame_id='map';g.pose.header.stamp=n.get_clock().now().to_msg()
  g.pose.pose.position.x=args.x;g.pose.pose.position.y=args.y
  g.pose.pose.orientation.z=math.sin(args.yaw/2);g.pose.pose.orientation.w=math.cos(args.yaw/2)
  sent=n.action.send_goal_async(g)
  assert n.spin_until(lambda:sent.done(),5),'goal send timeout'
  handle=sent.result();assert handle.accepted,'goal rejected'
  res=handle.get_result_async()
  if args.obstacle_after is not None:
   n.spin_until(lambda:False,args.obstacle_after)
   result['obstacle_added']=n.set_obstacle(True)
   result['obstacle_added_after_s']=args.obstacle_after
   if args.obstacle_clear_after is not None:
    n.spin_until(lambda:False,args.obstacle_clear_after)
    result['obstacle_removed']=n.set_obstacle(False)
  if args.cancel_after is not None:
   n.spin_until(lambda:False,args.cancel_after)
   canceled=handle.cancel_goal_async()
   assert n.spin_until(lambda:canceled.done(),5),'cancel request timeout'
  if not n.spin_until(lambda:res.done(),args.timeout):
   result['timeout']=True
   canceled=handle.cancel_goal_async()
   n.spin_until(lambda:canceled.done(),5)
   n.spin_until(lambda:res.done(),5)
  assert res.done(),'navigation result unavailable after cancel'
  result['goal_status']=res.result().status
  result['goal_succeeded']=res.result().status==GoalStatus.STATUS_SUCCEEDED
  result['command_count']=len(n.commands)
  result['nonzero_commands']=sum(abs(x[1])+abs(x[2])>.01 for x in n.commands)
  if args.cancel_after is not None:
   n.spin_until(lambda:False,2.)
  result['trace']=n.trace
  result['commands']=n.commands
  print(json.dumps({k:v for k,v in result.items() if k not in ('trace','commands')}))
 finally:
  if n.obstacle_active:result['obstacle_removed']=n.set_obstacle(False)
  n.set_nav_until(False,lambda:not n.ready,.5);n.manual.publish(Twist())
  n.spin_until(lambda:False,.5)
  result['trace']=n.trace
  result['commands']=n.commands
  result['final_confidence']=n.conf
  result['final_ready']=n.ready
  result['controller_mode']=n.controller_mode()
  result['plan_updates']=n.plan_updates
  result['min_obstacle_distance_m']=n.min_obstacle_distance
  global_samples=[]
  for sample in n.trace:
   if len(sample)>=10 and sample[8] is not None:
    global_samples.append((sample[1],sample[2],sample[8],sample[9]))
  if global_samples:
   global_errors=[math.hypot(mx-tx,my-ty) for tx,ty,mx,my in global_samples]
   final=global_samples[-1]
   result['final_truth_xy']=[final[0],final[1]]
   result['final_map_xy']=[final[2],final[3]]
   result['final_map_error_xy_m']=global_errors[-1]
   result['max_map_error_xy_m']=max(global_errors)
  Path(args.output).write_text(json.dumps(result))
  n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
