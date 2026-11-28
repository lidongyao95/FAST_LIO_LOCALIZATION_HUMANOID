"""Start LIO and 3-D global localization against the walking MuJoCo scene.

Defaults target the MuJoCo scene. For real FAST-LIO use ``real_robot:=true``,
``start_lio:=false``, ``use_sim_time:=false`` and the measured initial alignment.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from typing import List
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    repo=LaunchConfiguration('repo')
    map_ply=LaunchConfiguration('map_ply')
    show_rviz=LaunchConfiguration('rviz')
    start_lio=LaunchConfiguration('start_lio')
    real_robot=LaunchConfiguration('real_robot')
    use_sim_time=ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)
    publish_motion_tf=ParameterValue(LaunchConfiguration('publish_motion_tf'),value_type=bool)
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
        DeclareLaunchArgument('rviz',default_value='false',
                              description='Open RViz; the navigation launch already provides one'),
        DeclareLaunchArgument('use_sim_time',default_value='true',
                              description='Sim clock; pass false on the real robot'),
        DeclareLaunchArgument('start_lio',default_value='true',
                              description='Include FAST-LIO; pass false when it already runs'),
        DeclareLaunchArgument('real_robot',default_value='false',
                              description='Use FAST-LIO body TF without the simulated base or LiDAR TF'),
        DeclareLaunchArgument('lio_config_file',default_value='g1_walking.yaml'),
        DeclareLaunchArgument('publish_motion_tf',default_value='false',
                              description='Publish additional map -> motion_link TF'),
        DeclareLaunchArgument('loc_frequence',default_value='0.5',
                              description='Registration period in seconds; smaller publishes map -> odom faster'),
        *[DeclareLaunchArgument(k,default_value=v) for k,v in
          [('initial_x','0.0'),('initial_y','0.0'),('initial_z','0.793'),
           ('initial_roll','0.0'),('initial_pitch','0.0'),('initial_yaw','0.0')]],
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('fast_lio'),'launch','mapping.launch.py'])),
            condition=IfCondition(start_lio),
            launch_arguments={'config_path':PathJoinSubstitution([repo,'packages/fast_lio/config']),
                              'config_file':LaunchConfiguration('lio_config_file'),
                              'use_sim_time':LaunchConfiguration('use_sim_time'),'rviz':'false',
                              'debug_log_dir':PathJoinSubstitution([repo,'sim/generated/logs'])}.items()),
        Node(package='open3d_loc',executable='global_localization_node',
             name='global_localization_node',output='screen',
             remappings=[('/map','/localization/map_cloud'),('/scan','/localization/scan_cloud')],
             parameters=[{'use_sim_time':use_sim_time,'path_map':map_ply,
                'initialpose':initial,
                'pcd_queue_maxsize':5,'voxelsize_coarse':.2,'voxelsize_fine':.08,
                'threshold_fitness_init':.5,'threshold_fitness':.5,
                'loc_frequence':ParameterValue(LaunchConfiguration('loc_frequence'),value_type=float),'maxpoints_source':20000,'maxpoints_target':200000,
                'filter_odom2map':False,'publish_motion_tf':publish_motion_tf,
                'dis_updatemap':1.,'confidence_loc_th':.6}]),
        Node(package='rviz2',executable='rviz2',name='g1_walking_rviz',output='screen',
             arguments=['-d',PathJoinSubstitution([repo,'shared/nav/navigation.rviz'])],
             parameters=[{'use_sim_time':use_sim_time}],
             condition=IfCondition(show_rviz)),
        # Positional arguments keep this working on Foxy, which rejects --frame-id.
        Node(package='tf2_ros',executable='static_transform_publisher',name='odom_to_camera_init',
             arguments=['0','0','0','0','0','0','odom','camera_init']),
        Node(package='tf2_ros',executable='static_transform_publisher',name='base_link_to_imu_link',
             arguments=['0','0','0','0','0','0','base_link','imu_link']),
        Node(package='tf2_ros',executable='static_transform_publisher',name='base_link_to_motion_link',
             arguments=['0','0','0','0','0','0','base_link','motion_link']),
        Node(package='tf2_ros',executable='static_transform_publisher',name='imu_link_to_lidar_link',
             condition=UnlessCondition(real_robot),
             arguments=['0','0','0.45','0','0','0','imu_link','lidar_link']),
        ExecuteProcess(cmd=['/usr/bin/python3',PathJoinSubstitution([repo,'sim/nav_tf.py']),
                            '--ros-args','-p',['use_sim_time:=',LaunchConfiguration('use_sim_time')]],
                        condition=UnlessCondition(real_robot),output='screen'),
    ]
    return LaunchDescription(actions)
