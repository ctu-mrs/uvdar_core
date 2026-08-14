from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("uvdar_core"),
                "config",
                "default.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value=EnvironmentVariable("UAV_NAME", default_value="uav1"),
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value=EnvironmentVariable("USE_SIM_TIME", default_value="false"),
        ),
        Node(
            package="uvdar_core",
            executable="pose_estimator_node",
            name="pose_estimator",
            namespace=namespace,
            output="screen",
            parameters=[{"config_path": config_file, "use_sim_time": use_sim_time}],
        ),
    ])
