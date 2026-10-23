#!/usr/bin/env python3
"""ROS commands -> atomic local IPC -> upstream G1 controller. Simulation only."""
import os,time,math
from pathlib import Path
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool, Float32
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
ROOT=Path(__file__).resolve().parents[1]
class Commands(Node):
 def __init__(self):
  super().__init__('g1_command_bridge');self.mode=1;self.vel=[0.,0.,0.];self.last=0.
  self.nav_vel=[0.,0.,0.];self.nav_last=0.;self.nav_enabled=False
  self.confidence=0.;self.conf_last=0.;self.odom_last=0.;self.released=False;self.release_last=0.
  self.domain=os.environ.get('ROS_DOMAIN_ID','72')
  self.path=ROOT/f'.deps/g1_command-{self.domain}.txt'
  self.status_path=ROOT/f'.deps/g1-controller/status-{self.domain}.txt'
  self.create_subscription(Twist,'/cmd_vel',self.command,10)
  self.create_subscription(Twist,'/cmd_vel_nav',self.nav_command,10)
  self.create_subscription(String,'/g1/controller_mode',self.mode_cb,10)
  self.create_subscription(Bool,'/g1/nav_enable',self.nav_enable_cb,10)
  self.create_subscription(Bool,'/g1/released',self.release_cb,10)
  self.create_subscription(Float32,'/localization_3d_confidence',self.conf_cb,10)
  self.create_subscription(Odometry,'/odom',lambda msg:setattr(self,'odom_last',time.monotonic()),10)
  self.ready_pub=self.create_publisher(Bool,'/g1/nav_ready',10)
  self.create_timer(.02,self.write)
 def mode_cb(self,msg):
  modes={'passive':1,'stand':2,'velocity':3}
  if msg.data in modes:self.mode=modes[msg.data];self.vel=[0.,0.,0.];self.last=0.
 def command(self,msg):
  v=[msg.linear.x,msg.linear.y,msg.angular.z]
  if all(math.isfinite(x) for x in v):self.vel=[max(-l,min(l,x)) for x,l in zip(v,[.3,.15,.4])];self.last=time.monotonic()
 def nav_command(self,msg):
  v=[msg.linear.x,msg.linear.y,msg.angular.z]
  if all(math.isfinite(x) for x in v):self.nav_vel=[max(-l,min(l,x)) for x,l in zip(v,[.3,.15,.4])];self.nav_last=time.monotonic()
 def nav_enable_cb(self,msg):
  self.nav_enabled=msg.data;self.nav_last=0.
 def conf_cb(self,msg):
  self.confidence=msg.data;self.conf_last=time.monotonic()
 def release_cb(self,msg):
  self.released=msg.data;self.release_last=time.monotonic()
 def controller_ready(self):
  try:
   stamp,mode=self.status_path.read_text().split()[:2]
   return time.time()-float(stamp)<.5 and int(mode)==3
  except (OSError,ValueError,IndexError):return False
 def write(self):
  now=time.monotonic()
  ready=(self.nav_enabled and self.mode==3 and self.controller_ready() and
         self.released and now-self.release_last<.4 and
         now-self.odom_last<.4 and now-self.conf_last<.8 and self.confidence>=.6)
  b=Bool();b.data=ready;self.ready_pub.publish(b)
  if self.nav_enabled:v=self.nav_vel if ready and now-self.nav_last<.3 else [0.,0.,0.]
  else:v=self.vel if now-self.last<.3 and self.mode==3 else [0.,0.,0.]
  source=1 if self.nav_enabled else 0
  tmp=self.path.with_name(self.path.name+f'.tmp-{os.getpid()}')
  tmp.write_text(' '.join(map(str,[time.time(),self.mode,*v,source]))+'\n')
  tmp.replace(self.path)
def main():
 rclpy.init();n=Commands()
 try:rclpy.spin(n)
 except KeyboardInterrupt:pass
 finally:n.mode=1;n.vel=[0.,0.,0.];n.write();n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
