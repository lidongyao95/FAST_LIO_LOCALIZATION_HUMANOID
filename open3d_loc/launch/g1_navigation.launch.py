"""Nav2 map, planner, controller and RViz for the walking G1."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,TimerAction,ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    repo=LaunchConfiguration('repo');params=LaunchConfiguration('params_file')
    map_yaml=LaunchConfiguration('map_yaml')
    rviz=LaunchConfiguration('rviz')
    actions=[
        DeclareLaunchArgument('repo',description='Absolute repository path'),
        DeclareLaunchArgument('params_file',default_value=PathJoinSubstitution([repo,'navigation/g1_nav2_params.yaml'])),
        DeclareLaunchArgument('map_yaml',default_value=PathJoinSubstitution([repo,'sim/generated/room.yaml'])),
        DeclareLaunchArgument('rviz',default_value='true'),
        Node(package='nav2_map_server',executable='map_server',name='map_server',output='screen',
             parameters=[params,{'use_sim_time':True,'yaml_filename':map_yaml}]),
    ]
    for pkg,exe,name in [('nav2_controller','controller_server','controller_server'),
                         ('nav2_planner','planner_server','planner_server'),
                         ('nav2_behaviors','behavior_server','behavior_server'),
                         ('nav2_bt_navigator','bt_navigator','bt_navigator')]:
        remap=[('cmd_vel','cmd_vel_nav')] if name in ('controller_server','behavior_server') else []
        actions.append(Node(package=pkg,executable=exe,name=name,output='screen',
                            parameters=[params,{'use_sim_time':True}],remappings=remap))
    actions.extend([
        Node(package='rviz2',executable='rviz2',name='g1_navigation_rviz',output='screen',
             arguments=['-d',PathJoinSubstitution([FindPackageShare('open3d_loc'),'rviz_cfg','g1_navigation.rviz'])],
             parameters=[{'use_sim_time':True}],condition=IfCondition(rviz)),
        TimerAction(period=1.,actions=[ExecuteProcess(
            cmd=['/usr/bin/python3',PathJoinSubstitution([repo,'scripts/ensure_g1_nav_lifecycle.py'])],
            output='screen')]),
    ])
    return LaunchDescription(actions)
