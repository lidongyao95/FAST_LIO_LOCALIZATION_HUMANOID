"""G1 supported-base sensor demo; independent of external simulation workspaces."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    root = LaunchConfiguration('repo')
    headless = LaunchConfiguration('headless')
    nodes = [
        DeclareLaunchArgument('repo', description='Absolute path to the source checkout'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('fast_lio'), 'launch', 'mapping.launch.py'])),
            launch_arguments={'config_file': 'g1_mujoco.yaml',
                              'debug_log_dir': PathJoinSubstitution([root, 'sim/generated/logs'])}.items()),
        Node(package='open3d_loc', executable='global_localization_node',
             name='global_localization_node', output='screen', parameters=[{
                 'path_map': PathJoinSubstitution([root, 'sim/generated/room.ply']),
                 'initialpose': [0., 0., .793, 0., 0., 0.],
                 'pcd_queue_maxsize': 5, 'voxelsize_coarse': .2, 'voxelsize_fine': .08,
                 'threshold_fitness_init': .5, 'threshold_fitness': .5,
                 'loc_frequence': 2., 'maxpoints_source': 20000,
                 'maxpoints_target': 200000, 'filter_odom2map': False,
                 'dis_updatemap': 1., 'confidence_loc_th': .6,
             }]),
        Node(package='rviz2', executable='rviz2', name='g1_localization_rviz',
             arguments=['-d', PathJoinSubstitution([FindPackageShare('open3d_loc'),
                                                    'rviz_cfg', 'g1_mujoco.rviz'])],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]
    # Frames match FAST-LIO's pelvis IMU, not simulator ground-truth odometry.
    for parent, child in [('odom', 'camera_init'), ('body', 'imu_link'),
                          ('imu_link', 'base_link'), ('base_link', 'motion_link')]:
        nodes.append(Node(package='tf2_ros', executable='static_transform_publisher',
                          name=f'{parent}_to_{child}', arguments=[
                              '--frame-id', parent, '--child-frame-id', child]))
    # Separate conditions keep the sensor process command easy to reproduce.
    from launch.conditions import UnlessCondition
    for condition, extra in [(IfCondition(headless), ['--headless']),
                              (UnlessCondition(headless), [])]:
        nodes.append(ExecuteProcess(cmd=[PathJoinSubstitution([root, '.deps/venv/bin/python']),
                                         PathJoinSubstitution([root, 'scripts/g1_mujoco_sensors.py']),
                                         '--move', *extra], output='screen', condition=condition))
    return LaunchDescription(nodes)
