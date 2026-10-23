#!/usr/bin/env python3
"""Pin and adapt the installed ROS Humble Nav2 example to the G1 scene."""
from pathlib import Path
import yaml

root=Path(__file__).resolve().parents[1]
source=Path('/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml')
params=yaml.safe_load(source.read_text())
params.pop('amcl',None)
for node in ('bt_navigator','behavior_server'):
    params[node]['ros__parameters']['robot_base_frame']='base_footprint'
for name in ('local_costmap','global_costmap'):
    cost=params[name][name]['ros__parameters']
    cost['robot_base_frame']='base_footprint'
    cost.pop('robot_radius',None)
    cost['footprint']='[[0.34, 0.28], [0.34, -0.28], [-0.34, -0.28], [-0.34, 0.28]]'
    cost['transform_tolerance']=.3
    cost['plugins']=['static_layer','obstacle_layer','inflation_layer'] if name=='global_costmap' else ['obstacle_layer','inflation_layer']
    cost.pop('voxel_layer',None)
    obs=cost['obstacle_layer'] if name=='global_costmap' else {'plugin':'nav2_costmap_2d::ObstacleLayer','enabled':True}
    obs['observation_sources']='lidar'
    obs.pop('scan',None)
    obs['lidar']={'topic':'/livox/lidar','data_type':'PointCloud2','marking':True,'clearing':True,
                  'min_obstacle_height':.15,'max_obstacle_height':1.8,
                  'obstacle_min_range':.4,'obstacle_max_range':8.,
                  'raytrace_min_range':.4,'raytrace_max_range':9.}
    cost['obstacle_layer']=obs
    cost['inflation_layer']['inflation_radius']=.6
    cost['inflation_layer']['cost_scaling_factor']=4.
    cost['always_send_full_costmap']=True
    if name=='local_costmap':cost['width']=5;cost['height']=5
ctrl=params['controller_server']['ros__parameters']
ctrl['min_y_velocity_threshold']=.001
ctrl['progress_checker']['required_movement_radius']=.15
ctrl['progress_checker']['movement_time_allowance']=15.
ctrl['general_goal_checker']['xy_goal_tolerance']=.25
ctrl['general_goal_checker']['yaw_goal_tolerance']=.25
dwb=ctrl['FollowPath']
for key,value in {'max_vel_x':.2,'max_vel_theta':.3,'max_speed_xy':.2,
                  'acc_lim_x':.25,'decel_lim_x':-.25,'acc_lim_theta':.4,'decel_lim_theta':-.4,
                  'trans_stopped_velocity':.03,'sim_time':1.2,'vtheta_samples':15}.items():dwb[key]=value
params['behavior_server']['ros__parameters']['max_rotational_vel']=.3
params['behavior_server']['ros__parameters']['min_rotational_vel']=.1
params['behavior_server']['ros__parameters']['rotational_acc_lim']=.4
params['planner_server']['ros__parameters']['GridBased']['allow_unknown']=False
params['map_server']['ros__parameters']['yaml_filename']=str(root/'sim/generated/room.yaml')
out=root/'navigation/g1_nav2_params.yaml'
out.write_text('# Generated from installed ROS Humble nav2_bringup defaults by scripts/generate_nav_params.py\n'+yaml.safe_dump(params,sort_keys=False))
print(out)
