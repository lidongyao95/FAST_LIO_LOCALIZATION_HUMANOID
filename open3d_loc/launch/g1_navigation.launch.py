"""Nav2 map, planner, optional controller stack and RViz for the walking G1.

``mode:=full`` keeps the simulation stack (map, planner, controller, behaviors,
BT navigator). ``mode:=plan`` starts only ``map_server``, ``planner_server`` and
their lifecycle management. ``mode:=real`` adds the real-frame controller and
BT navigator; velocity reaches the SDK only through an explicitly started web session.
"""
import math
import subprocess
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


def rewrite_params(source,use_sim_time,map_yaml,robot_frame="",planning_obstacles=False,real_navigation=False,ground_z=0.):
 """Rewrite every node section so the launch arguments win over the shared file."""
 params=yaml.safe_load(Path(source).read_text())
 for section in params.values():
  stack=[section] if isinstance(section,dict) else []
  while stack:
   entry=stack.pop()
   if 'ros__parameters' in entry:entry['ros__parameters']['use_sim_time']=use_sim_time
   stack.extend(value for value in entry.values() if isinstance(value,dict))
 params.setdefault('map_server',{}).setdefault('ros__parameters',{})['yaml_filename']=map_yaml
 if planning_obstacles:
  costmap=params['global_costmap']['global_costmap']['ros__parameters']
  costmap['obstacle_layer']['lidar']['topic']='/g1/planning_obstacles/cloud'
  layer=costmap['obstacle_layer']
  layer['lidar']['observation_persistence']=0.0
  layer['lidar'].update(marking=True,clearing=False)
  layer['clearing']=dict(layer['lidar'],topic='/g1/planning_obstacles/clearing_cloud',marking=False,clearing=True)
  layer['observation_sources']='lidar clearing'
  costmap['clearable_layers']=['obstacle_layer']
  # Foxy's default planner ID redeclares its plugin parameter on reconfigure.
  planner=params['planner_server']['ros__parameters']
  planner['G1GridBased']=planner.pop('GridBased')
  planner['planner_plugins']=['G1GridBased']
 if robot_frame:
  params['global_costmap']['global_costmap']['ros__parameters']['robot_base_frame']=robot_frame
 if real_navigation:
  # Use the same map-height filtered obstacle streams in both costmaps.
  global_params=params['global_costmap']['global_costmap']['ros__parameters']
  global_params['obstacle_layer']['lidar'].update(min_obstacle_height=ground_z+.15,max_obstacle_height=ground_z+1.8)
  global_params['obstacle_layer']['clearing'].update(min_obstacle_height=ground_z+.15,max_obstacle_height=ground_z+1.8)
  local_params=params['local_costmap']['local_costmap']['ros__parameters']
  local_params.update(global_frame='map',robot_base_frame=robot_frame)
  local_params['plugins']=['static_layer','obstacle_layer','inflation_layer']
  # Foxy names this parameter in the singular.
  params['controller_server']['ros__parameters']['goal_checker_plugin']='general_goal_checker'
  local_params['obstacle_layer']=dict(global_params['obstacle_layer'])
  local_params['obstacle_layer']['lidar']=dict(global_params['obstacle_layer']['lidar'])
  local_params['obstacle_layer']['clearing']=dict(global_params['obstacle_layer']['clearing'])
  for name in ('bt_navigator','controller_server'):
   values=params[name]['ros__parameters']
   values['odom_topic']='/Odometry_loc'
   if name=='bt_navigator': values['robot_base_frame']=robot_frame
  navigator=params['bt_navigator']['ros__parameters']
  navigator['plugin_lib_names']=['nav2_compute_path_to_pose_action_bt_node','nav2_follow_path_action_bt_node',
                                 'nav2_rate_controller_bt_node','nav2_pipeline_sequence_bt_node']
 handle=tempfile.NamedTemporaryFile('w',prefix='g1_nav_params_',suffix='.yaml',delete=False)
 if real_navigation:
  tree=Path(handle.name).with_suffix('.xml')
  tree.write_text('<root main_tree_to_execute="MainTree"><BehaviorTree ID="MainTree"><PipelineSequence>'
                  '<RateController hz="1.0"><ComputePathToPose goal="{goal}" path="{path}" planner_id="G1GridBased"/>'
                  '</RateController><FollowPath path="{path}" controller_id="FollowPath"/>'
                  '</PipelineSequence></BehaviorTree></root>')
  navigator['default_bt_xml_filename']=str(tree)
  navigator['default_nav_to_pose_bt_xml']=str(tree)
 yaml.safe_dump(params,handle,sort_keys=False)
 handle.close()
 return handle.name


def remove_on_shutdown(path):
 """Delete the rewritten parameter file once the launch shuts down."""
 def cleanup(context):
  Path(path).unlink(missing_ok=True)
  Path(path).with_suffix('.xml').unlink(missing_ok=True)
  return []
 return RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)]))


def check_lifecycle_exit(event, context):
 if event.returncode != 0 and not context.is_shutdown:
  reason=f'G1 Nav2 lifecycle startup failed (exit {event.returncode})'
  # Foxy reports callback exceptions without shutting down running processes.
  context.emit_event_sync(Shutdown(reason=reason))
  raise RuntimeError(reason)


def check_obstacle_input_exit(event, context):
 if not context.is_shutdown:
  context.emit_event_sync(Shutdown(reason=f'Planning obstacle input exited ({event.returncode})'))


def launch_setup(context):
 repo=Path(LaunchConfiguration('repo').perform(context))
 mode=LaunchConfiguration('mode').perform(context)
 if mode not in ('full','plan','real'):
  raise ValueError(f'Invalid mode {mode!r}; expected full, plan or real')
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
 if mode=='real':
  if use_sim_time or not Path(map_yaml).is_file():raise ValueError('real navigation requires an existing map and use_sim_time:=false')
  nodes=['map_server','planner_server','controller_server','bt_navigator']
 if mode=='plan':
  forbidden=[name for name in FULL_NODES if name not in nodes]
  actions=[LogInfo(msg='G1 Nav2 plan-only: skipping '+', '.join(forbidden))]
 else:
  actions=[]
 robot_frame=LaunchConfiguration('robot_frame').perform(context)
 if mode=='real':robot_frame=robot_frame or 'body'
 if robot_frame and mode=='full':
  raise ValueError('robot_frame override is only supported in real-robot modes')
 subprocess.run(['/usr/bin/python3',str(repo/'scripts/env/g1_processes.py'),
                 '--check','--scope','navigation','--repo',str(repo)],check=True,timeout=5)
 subprocess.run(['/usr/bin/python3',str(repo/'scripts/navigation/ensure_g1_nav_lifecycle.py'),
                 '--wait-for-empty-graph'],check=True,timeout=12)
 ground_z=float(LaunchConfiguration('ground_z').perform(context)) if mode=='real' else 0.
 if not math.isfinite(ground_z):raise ValueError('ground_z must be finite')
 params=rewrite_params(LaunchConfiguration('params_file').perform(context),use_sim_time,map_yaml,
                       robot_frame,planning_obstacles=mode in ('plan','real'),real_navigation=mode=='real',
                       ground_z=ground_z)
 actions.append(LogInfo(msg=f'G1 Nav2 mode={mode} use_sim_time={use_sim_time} map={map_yaml} params={params}'))
 actions.append(remove_on_shutdown(params))
 if mode in ('plan','real'):
  obstacle_input=ExecuteProcess(
      cmd=['/usr/bin/python3',str(repo/'scripts/navigation/planning_obstacles.py'),
           '--ros-args','-p',f'use_sim_time:={str(use_sim_time).lower()}'],output='screen')
  actions.append(RegisterEventHandler(OnProcessExit(target_action=obstacle_input,on_exit=check_obstacle_input_exit)))
  actions.append(obstacle_input)
 for name in nodes:
  package,executable=PACKAGES[name]
  remap=[('cmd_vel','/g1/navigation/cmd_vel' if mode=='real' else 'cmd_vel_nav')] if name in ('controller_server','behavior_server') else []
  actions.append(Node(package=package,executable=executable,name=name,output='screen',
                      parameters=[params],remappings=remap))
 actions.append(Node(package='rviz2',executable='rviz2',name='g1_navigation_rviz',output='screen',
                     arguments=['-d',PathJoinSubstitution([FindPackageShare('open3d_loc'),'rviz_cfg','g1_navigation.rviz'])],
                     parameters=[{'use_sim_time':use_sim_time}],condition=IfCondition(LaunchConfiguration('rviz'))))
 lifecycle=ExecuteProcess(
     cmd=['/usr/bin/python3',str(repo/'scripts/navigation/ensure_g1_nav_lifecycle.py'),*nodes],
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
         description='full for simulation, plan for planning only, real for web-owned real navigation'),
     DeclareLaunchArgument('robot_frame',default_value='',
         description='Optional global costmap robot frame in plan mode'),
     DeclareLaunchArgument('ground_z',default_value='0.0',description='real mode: map floor height from export metadata'),
     DeclareLaunchArgument('rviz',default_value='true'),
     OpaqueFunction(function=launch_setup),
 ])
