"""Start LIO and 3-D global localization against the walking MuJoCo scene."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from typing import List
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    repo=LaunchConfiguration('repo')
    map_ply=LaunchConfiguration('map_ply')
    show_rviz=LaunchConfiguration('show_rviz')
    keys=('initial_x','initial_y','initial_z','initial_roll','initial_pitch','initial_yaw')
    initial_parts=['[']
    for i,key in enumerate(keys):
        if i:initial_parts.append(', ')
        initial_parts.append(LaunchConfiguration(key))
    initial_parts.append(']')
    initial=ParameterValue(initial_parts,value_type=List[float])
    actions=[
        DeclareLaunchArgument('repo',description='Absolute repository path'),
        DeclareLaunchArgument('map_ply',default_value=PathJoinSubstitution([repo,'sim/generated/room.ply'])),
        DeclareLaunchArgument('show_rviz',default_value='true'),
        *[DeclareLaunchArgument(k,default_value=v) for k,v in
          [('initial_x','0.0'),('initial_y','0.0'),('initial_z','0.793'),
           ('initial_roll','0.0'),('initial_pitch','0.0'),('initial_yaw','0.0')]],
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('fast_lio'),'launch','mapping.launch.py'])),
            launch_arguments={'config_path':PathJoinSubstitution([repo,'FAST_LIO/config']),
                              'config_file':'g1_walking.yaml','use_sim_time':'true','rviz':'false',
                              'debug_log_dir':PathJoinSubstitution([repo,'sim/generated/logs'])}.items()),
        Node(package='open3d_loc',executable='global_localization_node',
             name='global_localization_node',output='screen',
             remappings=[('/map','/localization/map_cloud'),('/scan','/localization/scan_cloud')],
             parameters=[{'use_sim_time':True,'path_map':map_ply,
                'initialpose':initial,
                'pcd_queue_maxsize':5,'voxelsize_coarse':.2,'voxelsize_fine':.08,
                'threshold_fitness_init':.5,'threshold_fitness':.5,
                'loc_frequence':.5,'maxpoints_source':20000,'maxpoints_target':200000,
                'filter_odom2map':False,'publish_motion_tf':False,
                'dis_updatemap':1.,'confidence_loc_th':.6}]),
        Node(package='rviz2',executable='rviz2',name='g1_walking_rviz',output='screen',
             arguments=['-d',PathJoinSubstitution([
                 FindPackageShare('open3d_loc'),'rviz_cfg','g1_navigation.rviz'])],
             parameters=[{'use_sim_time':True}],
             condition=IfCondition(show_rviz)),
        Node(package='tf2_ros',executable='static_transform_publisher',name='odom_to_camera_init',
             arguments=['--frame-id','odom','--child-frame-id','camera_init']),
        Node(package='tf2_ros',executable='static_transform_publisher',name='base_link_to_imu_link',
             arguments=['--frame-id','base_link','--child-frame-id','imu_link']),
        Node(package='tf2_ros',executable='static_transform_publisher',name='base_link_to_motion_link',
             arguments=['--frame-id','base_link','--child-frame-id','motion_link']),
        Node(package='tf2_ros',executable='static_transform_publisher',name='imu_link_to_lidar_link',
             arguments=['--z','.45','--frame-id','imu_link','--child-frame-id','lidar_link']),
        ExecuteProcess(cmd=['/usr/bin/python3',PathJoinSubstitution([repo,'scripts/g1_nav_tf.py']),
                            '--ros-args','-p','use_sim_time:=true'],output='screen'),
    ]
    return LaunchDescription(actions)
