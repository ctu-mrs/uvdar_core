#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

print("UGGA", EnvironmentVariable('BLUEFOX_LEFT_ID'))
def generate_launch_description():
    declare_uav_name = DeclareLaunchArgument(
        'uav_name',
        default_value=EnvironmentVariable('UAV_NAME'),
        description='UAV namespace'
    )

    single_camera_launch = PathJoinSubstitution([
        FindPackageShare('uvdar_core'),
        'launch',
        'single_bluefox.launch.py'
    ])

    left_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(single_camera_launch),
        launch_arguments={
            'uav_name': LaunchConfiguration('uav_name'),
            'camera_name': 'left',
            'device': EnvironmentVariable('BLUEFOX_LEFT_ID'),
            'expose_us': EnvironmentVariable('EXPOSE_US_LEFT'),
            'aec': 'false',  # manual exposure - see note below
            'standalone': 'true',
        }.items()
    )

    right_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(single_camera_launch),
        launch_arguments={
            'uav_name': LaunchConfiguration('uav_name'),
            'camera_name': 'right',
            'device': EnvironmentVariable('BLUEFOX_RIGHT_ID'),
            'expose_us': EnvironmentVariable('EXPOSE_US_RIGHT'),
            'aec': 'false',
            'standalone': 'true',
        }.items()
    )

    return LaunchDescription([
        declare_uav_name,
        left_camera,
        right_camera,
    ])
