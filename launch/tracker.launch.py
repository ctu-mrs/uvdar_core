from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    namespace = LaunchConfiguration("namespace")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("uvdar_core"),
                "config",
                "default.yaml",
            ]),
        ),
        DeclareLaunchArgument("namespace", default_value=""),
        Node(
            package="uvdar_core",
            executable="tracker_node",
            name="tracker",
            namespace=namespace,
            output="screen",
            parameters=[{"config_path": config_file}],
        ),
    ])
