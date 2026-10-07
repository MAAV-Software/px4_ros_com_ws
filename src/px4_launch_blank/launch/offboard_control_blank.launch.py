#!/usr/bin/env python
"""Launch the px4_blank_launch

  ros2 launch px4_launch_blank offboard_control_blank.launch.py instance:=0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    instance_arg = DeclareLaunchArgument(
        'instance', default_value='0',
        description='Vehicle instance to target (0 or 1 in a 2-instance setup)')

    return LaunchDescription([
        instance_arg,
        Node(
            package='px4_launch_blank',
            executable='offboard_control_blank',
            name='offboard_control_blank',
            parameters=[{'vehicle_instance': ParameterValue(LaunchConfiguration('instance'), value_type=int)}],
            prefix='gnome-terminal --'
        ),
    ])
