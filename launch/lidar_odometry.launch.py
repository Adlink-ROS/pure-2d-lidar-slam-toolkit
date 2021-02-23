import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, GroupAction, DeclareLaunchArgument)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():

    rf2o_launch_dir = os.path.join(get_package_share_directory('rf2o_laser_odometry'), 'launch')
    slam_launch_dir = os.path.join(get_package_share_directory('neuronbot2_slam'), 'launch')

    pure_lidar_slam = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rf2o_launch_dir, '/rf2o_laser_odometry.launch.py']),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([slam_launch_dir, '/gmapping_launch.py']),
        ),
    ])

    ld = LaunchDescription()
    ld.add_action(pure_lidar_slam)
    return ld
