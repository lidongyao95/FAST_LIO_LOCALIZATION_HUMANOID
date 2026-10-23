#!/usr/bin/env python3
"""Configure and activate the Nav2 lifecycle nodes in dependency order."""
import time
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition

NODES=('map_server','controller_server','planner_server','behavior_server','bt_navigator')

class Repair(Node):
 def __init__(self):
  super().__init__('g1_nav_lifecycle_check')
  self.states={n:self.create_client(GetState,f'/{n}/get_state') for n in NODES}
  self.changes={n:self.create_client(ChangeState,f'/{n}/change_state') for n in NODES}
 def call(self,client,request,timeout=3.):
  if not client.wait_for_service(timeout_sec=timeout):return None
  future=client.call_async(request);end=time.monotonic()+timeout
  while not future.done() and time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.05)
  return future.result() if future.done() else None
 def state(self,name):
  r=self.call(self.states[name],GetState.Request(),1.)
  return r.current_state.id if r else None
 def change(self,name,transition):
  req=ChangeState.Request();req.transition.id=transition
  r=self.call(self.changes[name],req,5.)
  return bool(r and r.success)
 def run(self):
  deadline=time.monotonic()+40
  while time.monotonic()<deadline:
   states={name:self.state(name) for name in NODES}
   if all(s==3 for s in states.values()):
    self.get_logger().info('All G1 Nav2 lifecycle nodes active')
    return 0
   for name,state in states.items():
    if state==1:
     self.get_logger().info(f'Configuring {name}')
     self.change(name,Transition.TRANSITION_CONFIGURE)
     break
    if state==2:
     self.get_logger().info(f'Activating {name}')
     self.change(name,Transition.TRANSITION_ACTIVATE)
     break
   time.sleep(.5)
  self.get_logger().error('Nav2 lifecycle did not reach active state')
  return 1

def main():
 rclpy.init();node=Repair()
 try:raise SystemExit(node.run())
 finally:node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
