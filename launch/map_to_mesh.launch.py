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

    map2gazebo_launch_dir = os.path.join(get_package_share_directory('map2gazebo'))

    map2gazebo = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([map2gazebo_launch_dir, '/map2gazebo.launch.py']),
        )
    ])

    ld = LaunchDescription()
    ld.add_action(map2gazebo)
    return ld
