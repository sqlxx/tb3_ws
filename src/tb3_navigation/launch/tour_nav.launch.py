import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from nav2_common.launch import RewrittenYaml
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  use_sim_time = LaunchConfiguration('use_sim_time')
  map_yaml_file = LaunchConfiguration('map')
  # params_file = LaunchConfiguration("params_file", default=os.path.join(get_package_share_directory('tb3_navigation'), 'config', 'config.yaml'))
  log_level = LaunchConfiguration('log_level')

  param_substitutions = {
    'use_sim_time': use_sim_time,
    'yaml_filename': map_yaml_file
  }

  configured_params = ParameterFile(
    RewrittenYaml(
      source_file=os.path.join(get_package_share_directory('tb3_navigation'), 'config', 'config.yaml'),
      param_rewrites=param_substitutions,
      convert_types=True
    ), allow_substs=True
  )
  
  slam_lifecycle_nodes = ['map_saver']
  localization_lifecycle_nodes = ['map_server']

  return LaunchDescription([
    DeclareLaunchArgument(
      'use_sim_time',
      description='Use simulation time if true'
    ),
    DeclareLaunchArgument(
      'map',
      description="Path to map yaml file"
    ),
    DeclareLaunchArgument(
      'log_level',
      default_value='info'
    ),
    Node(
      package='nav2_lifecycle_manager',
      executable='lifecycle_manager',
      name='lifecycle_manager_slam',
      output='screen',
      arguments=['--ros-args', '--log-level', log_level],
      parameters=[{'use_sim_time': use_sim_time}, {'node_names': slam_lifecycle_nodes}, {'autostart': True}]
    ),
    Node(
      package='nav2_map_server',
      executable='map_saver_server',
      output='screen',
      respawn=False,
      respawn_delay=2.0,
      arguments=['--ros-args', '--log-level', log_level],
      parameters=[configured_params]
    ),
    Node(
      package='nav2_lifecycle_manager',
      executable='lifecycle_manager',
      name='lifecycle_manager_localization',
      output='screen',
      arguments=['--ros-args', '--log-level', log_level],
      parameters=[{'use_sim_time': use_sim_time}, {'node_names': localization_lifecycle_nodes}, {'autostart': True}]
    ),
    Node(
      package='nav2_map_server',
      executable='map_server',
      name='map_server',
      output='screen',
      parameters=[configured_params],
      arguments=['--ros-args', '--log-level', log_level]
    ),
    IncludeLaunchDescription(
      PathJoinSubstitution([FindPackageShare('tb3_slam'), 'launch', 'slam_toolbox_localization.launch.py']),
      launch_arguments={'use_sim_time': use_sim_time, 'log_level': log_level}.items()
    )
  ])

  

