#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Default path for files.
DEFAULT_PATH = os.path.join(
                get_package_share_directory('knapp_amr'),
                'yaml/')

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='knapp_amr',
            executable='order_optimizer',
            name='order_optimizer',
            parameters=[
                {"path" : DEFAULT_PATH}
            ],
            output='screen'),
    ])
