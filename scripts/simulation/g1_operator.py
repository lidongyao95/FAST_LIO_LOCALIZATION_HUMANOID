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

ROOT=Path(__file__).resolve().parents[2]
DOMAIN=os.environ.get('ROS_DOMAIN_ID','72')
STATUS_FILE=ROOT/f'.deps/g1-controller/status-{DOMAIN}.txt'
HOLD_TIMEOUT = .45
MANUAL_COMMANDS = {
 'w': (.15, 0., 0.), 's': (-.12, 0., 0.),
 'a': (0., .15, 0.), 'd': (0., -.15, 0.),
 'q': (0., 0., .2), 'e': (0., 0., -.2),
}

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
 def publish_mode_until(self,name,expected,seconds):
  end=time.monotonic()+seconds
  while time.monotonic()<end:
   self.publish_mode(name)
   rclpy.spin_once(self,timeout_sec=.05)
   if self.mode()==expected:return True
  return False
 def publish_enable_until(self,enabled,predicate,seconds):
  end=time.monotonic()+seconds
  while time.monotonic()<end:
   self.publish_enable(enabled)
   rclpy.spin_once(self,timeout_sec=.05)
   if predicate():return True
  return False
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
  assert self.publish_mode_until('stand',2,5),'FixStand transition failed'
  target=yaml.safe_load((ROOT/'.deps/g1-controller/config.yaml').read_text())['FSM']['FixStand']['qs'][-1]
  def settled():
   if not self.joints or len(self.joints.position)<len(target):return False
   error=math.sqrt(sum((a-b)**2 for a,b in zip(self.joints.position,target))/len(target))
   return error<.12
  assert self.until(settled,6),'stand joints did not settle'
  assert self.publish_mode_until('velocity',3,5),'walking policy transition failed'
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
  assert self.publish_enable_until(True,lambda:self.nav_ready,3),'navigation gate refused enable'
 def drive(self, direction, seconds):
  def ready():
   return (self.pose is not None and self.released and
           time.monotonic() - self.release_time < 1 and self.mode() == 3 and not self.nav_ready)
  if not self.until(ready, 3):
   raise RuntimeError('Released MuJoCo controller in manual velocity mode is required')
  cmd = Twist()
  cmd.linear.x, cmd.linear.y, cmd.angular.z = MANUAL_COMMANDS[direction]
  deadline = time.monotonic() + seconds
  try:
   while time.monotonic() < deadline:
    if not ready():
     raise RuntimeError('MuJoCo manual control lost during mapping')
    self.cmd_pub.publish(cmd)
    self.until(lambda: False, .08)
  finally:
   self.cmd_pub.publish(Twist())
   self.until(lambda: False, .2)
 def teleop(self):
  """Realtime single-key control: no seconds, no Enter needed."""
  import select,signal,sys,termios,tty
  if not sys.stdin.isatty():
   raise RuntimeError('teleop 需要交互终端；管道输入请用 drive 子命令')
  fd=sys.stdin.fileno();saved=termios.tcgetattr(fd)
  def ready():
   return (self.pose is not None and self.released and
           time.monotonic() - self.release_time < 1 and self.mode() == 3 and not self.nav_ready)
  if not self.until(ready, 3):
   raise RuntimeError('Released MuJoCo controller in manual velocity mode is required')
  def interrupt(signum,frame):raise KeyboardInterrupt
  previous={sig:signal.signal(sig,interrupt) for sig in (signal.SIGTERM,signal.SIGHUP)}
  cmd=Twist();moving=False;last_press=0.;current=None
  sys.stderr.write('实时控制：按住 w/a/s/d 移动、q/e 转向，松手即停；回车保存并结束\n')
  sys.stderr.flush()
  try:
   tty.setcbreak(fd)
   while True:
    readable,_,_=select.select([fd],[],[],.02)
    # 必须持续 spin，否则 /g1/released 与位姿回调不再更新，ready() 会很快失效。
    rclpy.spin_once(self,timeout_sec=.02)
    if readable:
     key=sys.stdin.read(1)
     if key in ('\r','\n'):break
     if key in (' ','x','X','\x1b'):
      cmd=Twist();moving=False;current=None
     elif key in MANUAL_COMMANDS:
      cmd.linear.x,cmd.linear.y,cmd.angular.z=MANUAL_COMMANDS[key];moving=True
      last_press=time.monotonic()
      if key!=current:
       current=key
       sys.stderr.write('按住 %s，松手即停\n' % key);sys.stderr.flush()
     else:
      continue
    elif moving and time.monotonic()-last_press>HOLD_TIMEOUT:
     # 终端长时间没有按键重复，说明键已松开：停止。
     cmd=Twist();moving=False;current=None
     sys.stderr.write('按键已松开，停止\n');sys.stderr.flush()
    if not ready():
     raise RuntimeError('MuJoCo manual control lost during mapping')
    self.cmd_pub.publish(cmd)
  finally:
   termios.tcsetattr(fd,termios.TCSADRAIN,saved)
   for sig,handler in previous.items():signal.signal(sig,handler)
   self.cmd_pub.publish(Twist());self.until(lambda:False,.2)
   del moving
 def stop(self):
  self.publish_enable_until(False,lambda:not self.nav_ready,.5)
  self.cmd_pub.publish(Twist())
  if self.cancel.wait_for_service(timeout_sec=.5):
   f=self.cancel.call_async(CancelGoal.Request());self.until(lambda:f.done(),3)
  self.until(lambda:False,.5)

def main():
 p=argparse.ArgumentParser()
 p.add_argument('action',choices=['prepare','enable-nav','stop','status','drive','teleop'])
 p.add_argument('direction',nargs='?',choices=MANUAL_COMMANDS)
 p.add_argument('--seconds',type=float)
 args=p.parse_args()
 if args.action=='drive':
  if args.direction is None or args.seconds is None or not 1 <= args.seconds <= 30:
   p.error('drive requires a direction and --seconds between 1 and 30')
 elif args.direction is not None or args.seconds is not None:
  p.error('direction and --seconds are only for drive')
 rclpy.init(args=[]);node=Operator()
 try:
  if args.action=='prepare':node.prepare()
  elif args.action=='enable-nav':node.enable_nav()
  elif args.action=='drive':node.drive(args.direction,args.seconds)
  elif args.action=='teleop':node.teleop()
  elif args.action=='stop':node.stop()
  else:
   node.until(lambda:node.pose is not None and node.joints is not None and
              node.conf_time>0 and node.odom_time>0,5)
  print(json.dumps(node.status()))
 finally:node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
