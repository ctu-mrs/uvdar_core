from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_detector = LaunchConfiguration("launch_detector")
    launch_tracker = LaunchConfiguration("launch_tracker")

    common_parameters = [{
        "config_path": config_file,
        "use_sim_time": use_sim_time,
    }]

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("uvdar_core"),
                "config",
                "default_bearing.yaml",
            ]),
            description="Detector, tracker, and bearing endpoint configuration",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value=EnvironmentVariable("UAV_NAME", default_value="uav1"),
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value=EnvironmentVariable("USE_SIM_TIME", default_value="false"),
        ),
        DeclareLaunchArgument(
            "launch_detector",
            default_value="true",
            description="Start the detector stage of the bearing endpoint",
        ),
        DeclareLaunchArgument(
            "launch_tracker",
            default_value="true",
            description="Start the tracker stage of the bearing endpoint",
        ),
        Node(
            package="uvdar_core",
            executable="detector_node",
            name="detector",
            namespace=namespace,
            output="screen",
            parameters=common_parameters,
            condition=IfCondition(launch_detector),
        ),
        Node(
            package="uvdar_core",
            executable="tracker_node",
            name="tracker",
            namespace=namespace,
            output="screen",
            parameters=common_parameters,
            condition=IfCondition(launch_tracker),
        ),
        Node(
            package="uvdar_core",
            executable="bearing_node",
            name="bearing",
            namespace=namespace,
            output="screen",
            parameters=common_parameters,
        ),
    ])

