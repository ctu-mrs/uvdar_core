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
                "calib_default.yaml",
            ]),
            description="Path to the complete calibrator YAML configuration",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value=EnvironmentVariable("UAV_NAME", default_value="uav1"),
            description="ROS namespace for the calibrator node",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value=EnvironmentVariable(
                "USE_SIM_TIME", default_value="false"
            ),
            description="Use the ROS simulation clock",
        ),
        Node(
            package="uvdar_core",
            executable="calibrator_node",
            name="calibrator",
            namespace=namespace,
            output="screen",
            emulate_tty=True,
            parameters=[{
                "config_path": config_file,
                "use_sim_time": use_sim_time,
            }],
        ),
    ])
