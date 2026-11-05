"""Nav2 map, planner, optional controller stack and RViz for the walking G1.

``mode:=full`` keeps the simulation stack (map, planner, controller, behaviors,
BT navigator). ``mode:=plan`` starts only ``map_server``, ``planner_server`` and
their lifecycle management, which is what the real robot C stage allows.
"""
import tempfile
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

FULL_NODES=['map_server','controller_server','planner_server','behavior_server','bt_navigator']
PLAN_NODES=['map_server','planner_server']
PACKAGES={'map_server':('nav2_map_server','map_server'),
          'controller_server':('nav2_controller','controller_server'),
          'planner_server':('nav2_planner','planner_server'),
          'behavior_server':('nav2_behaviors','behavior_server'),
          'bt_navigator':('nav2_bt_navigator','bt_navigator')}


def rewrite_params(source,use_sim_time,map_yaml):
 """Rewrite every node section so the launch arguments win over the shared file."""
 params=yaml.safe_load(Path(source).read_text())
 for section in params.values():
  stack=[section] if isinstance(section,dict) else []
  while stack:
   entry=stack.pop()
   if 'ros__parameters' in entry:entry['ros__parameters']['use_sim_time']=use_sim_time
   stack.extend(value for value in entry.values() if isinstance(value,dict))
 params.setdefault('map_server',{}).setdefault('ros__parameters',{})['yaml_filename']=map_yaml
 handle=tempfile.NamedTemporaryFile('w',prefix='g1_nav_params_',suffix='.yaml',delete=False)
 yaml.safe_dump(params,handle,sort_keys=False)
 handle.close()
 return handle.name


def remove_on_shutdown(path):
 """Delete the rewritten parameter file once the launch shuts down."""
 def cleanup(context):
  Path(path).unlink(missing_ok=True)
  return []
 return RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)]))


def check_lifecycle_exit(event, context):
 if event.returncode != 0 and not context.is_shutdown:
  reason=f'G1 Nav2 lifecycle startup failed (exit {event.returncode})'
  # Foxy reports callback exceptions without shutting down running processes.
  context.emit_event_sync(Shutdown(reason=reason))
  raise RuntimeError(reason)


def launch_setup(context):
 repo=Path(LaunchConfiguration('repo').perform(context))
 mode=LaunchConfiguration('mode').perform(context)
 if mode not in ('full','plan'):
  raise ValueError(f'Invalid mode {mode!r}; expected full or plan')
 clock=LaunchConfiguration('use_sim_time').perform(context).lower()
 map_yaml=LaunchConfiguration('map_yaml').perform(context)
 if mode=='plan' and (not map_yaml or not clock):
  raise ValueError('mode:=plan requires explicit map_yaml and use_sim_time')
 clock=clock or 'true'
 if clock not in ('true','false','1','0'):
  raise ValueError(f'Invalid use_sim_time {clock!r}; expected true or false')
 use_sim_time=clock in ('true','1')
 map_yaml=map_yaml or str(repo/'sim/generated/room.yaml')
 nodes=PLAN_NODES if mode=='plan' else FULL_NODES
 if mode=='plan':
  forbidden=[name for name in FULL_NODES if name not in nodes]
  actions=[LogInfo(msg='G1 Nav2 plan-only: skipping '+', '.join(forbidden))]
 else:
  actions=[]
 params=rewrite_params(LaunchConfiguration('params_file').perform(context),use_sim_time,map_yaml)
 actions.append(LogInfo(msg=f'G1 Nav2 mode={mode} use_sim_time={use_sim_time} map={map_yaml} params={params}'))
 actions.append(remove_on_shutdown(params))
 for name in nodes:
  package,executable=PACKAGES[name]
  remap=[('cmd_vel','cmd_vel_nav')] if name in ('controller_server','behavior_server') else []
  actions.append(Node(package=package,executable=executable,name=name,output='screen',
                      parameters=[params],remappings=remap))
 actions.append(Node(package='rviz2',executable='rviz2',name='g1_navigation_rviz',output='screen',
                     arguments=['-d',PathJoinSubstitution([FindPackageShare('open3d_loc'),'rviz_cfg','g1_navigation.rviz'])],
                     parameters=[{'use_sim_time':use_sim_time}],condition=IfCondition(LaunchConfiguration('rviz'))))
 lifecycle=ExecuteProcess(
     cmd=['/usr/bin/python3',str(repo/'scripts/ensure_g1_nav_lifecycle.py'),*nodes],
     output='screen')
 actions.append(RegisterEventHandler(OnProcessExit(target_action=lifecycle,on_exit=check_lifecycle_exit)))
 actions.append(TimerAction(period=1.,actions=[lifecycle]))
 return actions


def generate_launch_description():
 return LaunchDescription([
     DeclareLaunchArgument('repo',description='Absolute repository path'),
     DeclareLaunchArgument('params_file',default_value=PathJoinSubstitution(
         [LaunchConfiguration('repo'),'navigation/g1_nav2_params.yaml'])),
     DeclareLaunchArgument('map_yaml',default_value='',
         description='Required in plan mode; full defaults to sim/generated/room.yaml'),
     DeclareLaunchArgument('use_sim_time',default_value='',
         description='Required in plan mode; full defaults to true'),
     DeclareLaunchArgument('mode',default_value='full',
         description='full for simulation, plan for map_server plus planner_server only'),
     DeclareLaunchArgument('rviz',default_value='true'),
     OpaqueFunction(function=launch_setup),
 ])
