"""Build a FAST-LIO PCD map from the free-walking G1 sensors."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    repo=LaunchConfiguration('repo')
    rviz=LaunchConfiguration('rviz')
    return LaunchDescription([
        DeclareLaunchArgument('repo',description='Absolute repository path'),
        DeclareLaunchArgument('rviz',default_value='true'),
        Node(package='fast_lio',executable='fastlio_mapping',name='g1_mapping',output='screen',
             parameters=[PathJoinSubstitution([repo,'FAST_LIO/config/g1_mapping.yaml']),
                         {'use_sim_time':True,
                          'map_file_path':PathJoinSubstitution([repo,'sim/generated/mapping/map.pcd']),
                          'debug_log_dir':PathJoinSubstitution([repo,'sim/generated/logs'])}]),
        Node(package='rviz2',executable='rviz2',name='g1_mapping_rviz',output='screen',
             arguments=['-d',PathJoinSubstitution([
                 FindPackageShare('fast_lio'),'rviz','fastlio.rviz'])],
             parameters=[{'use_sim_time':True}],
             condition=IfCondition(rviz)),
    ])
