
#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
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
    uvcam_left_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=['uvcam_left_tf_', LaunchConfiguration('uav_name')],
        arguments=[
            '--x', '0.03',
            '--y', '0.10',
            '--z', '0.06',
            '--yaw', '-0.3490658504',
            '--pitch', '0.0',
            '--roll', '-1.57079632679',
            '--frame-id', [LaunchConfiguration('uav_name'), '/fcu'],
            '--child-frame-id', [LaunchConfiguration('uav_name'), '/bluefox_left'],
        ]
    )
    uvcam_right_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=['uvcam_right_tf_', LaunchConfiguration('uav_name')],
        arguments=[
            '--x', '0.03',
            '--y', '-0.10',
            '--z', '0.06',
            '--yaw', '-2.792526803',
            '--pitch', '0.0',
            '--roll', '-1.57079632679',
            '--frame-id', [LaunchConfiguration('uav_name'), '/fcu'],
            '--child-frame-id', [LaunchConfiguration('uav_name'), '/bluefox_right'],
        ]
    )
    return LaunchDescription([
        declare_uav_name,
        left_camera,
        right_camera,
        uvcam_left_tf,
        uvcam_right_tf,
    ])
