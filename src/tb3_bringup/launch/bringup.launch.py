from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
  return LaunchDescription([
    IncludeLaunchDescription(
      PathJoinSubstitution([FindPackageShare('tb3_slam_toolbox'), 'launch', 'slam_toolbox.launch.py'])
    ),
    IncludeLaunchDescription(
      PathJoinSubstitution([FindPackageShare('tb3_rosbridge'), 'launch', 'rosbridge.launch.py'])
    )
    
  ])