#!/usr/bin/env python3
"""Configure and activate the Nav2 lifecycle nodes in dependency order.

Node names may be passed as arguments; the default is the full simulation set.
"""
import os
from pathlib import Path
import sys
import time
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition

DEFAULT_NODES=('map_server','controller_server','planner_server','behavior_server','bt_navigator')
NODES=tuple(sys.argv[1:]) or DEFAULT_NODES

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
  states=dict.fromkeys(NODES)
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
  pending=', '.join(f'{name}={state if state is not None else "unreachable"}'
                    for name,state in states.items() if state!=3)
  self.get_logger().error(f'Nav2 lifecycle did not reach active state. Last pending states: {pending}')
  return 1

def local_navigation_processes():
 """Use the same checkout/domain process selection as the CLI preflight."""
 sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
 from processes import scan
 return [str(p.pid) for p in scan(Path('/proc'),Path(__file__).resolve().parents[2],
                                 int(os.environ.get('ROS_DOMAIN_ID','0')),'navigation')]


def wait_for_previous_session():
 node=rclpy.create_node('g1_nav_graph_preflight')
 deadline=time.monotonic()+8
 clear_since=None
 try:
  while time.monotonic()<deadline:
   rclpy.spin_once(node,timeout_sec=.1)
   names={name for name,namespace in node.get_node_names_and_namespaces()
          if namespace=='/'}
   conflicts=names.intersection((*DEFAULT_NODES,'recoveries_server','g1_planning_obstacles'))
   publishers=node.count_publishers('/global_costmap/costmap_raw')
   processes=local_navigation_processes()
   if conflicts or publishers or processes:
    clear_since=None
   elif clear_since is None:
    clear_since=time.monotonic()
   elif time.monotonic()-clear_since>=2:
    return 0
  node.get_logger().error(f'Previous navigation session still visible: nodes={sorted(conflicts)}, '
                          f'global costmap publishers={publishers}, local PIDs={processes}')
  return 1
 finally:
  node.destroy_node()


def main():
 rclpy.init()
 if sys.argv[1:]==['--wait-for-empty-graph']:
  try:raise SystemExit(wait_for_previous_session())
  finally:rclpy.try_shutdown()
 node=Repair()
 try:raise SystemExit(node.run())
 finally:node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
