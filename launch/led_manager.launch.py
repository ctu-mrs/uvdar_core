from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sequence_file = LaunchConfiguration("sequence_file")
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    # Points at the serial driver's inbound topic for the UVDAR LED board
    # (mirrors the ROS1 remap of ~baca_protocol_out to serial_uvdar/send_message).
    serial_send_topic = LaunchConfiguration("serial_send_topic")
    led_serial_port = LaunchConfiguration("led_serial_port")

    return LaunchDescription([
        DeclareLaunchArgument(
            "sequence_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("uvdar_core"),
                "config",
                "selected.txt",
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
        DeclareLaunchArgument(
            "serial_send_topic",
            default_value="serial_uvdar/send_message",
        ),
        DeclareLaunchArgument(
            "led_serial_port",
            default_value="/dev/MRS_MODULE1",
            description="Serial port the UVDAR LED board is connected to",
        ),
        # The LED manager only publishes BacaProtocol commands - this is the
        # driver that actually writes them out over UART to the LED board
        # (mirrors the ROS1 launch file, which started serial_uvdar alongside
        # uvdar_led_manager_node).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("mrs_serial"),
                    "launch",
                    "baca_protocol.launch.py",
                ])
            ]),
            launch_arguments={
                "UAV_NAME": namespace,
                "node_name": "serial_uvdar",
                "portname": led_serial_port,
            }.items(),
        ),
        Node(
            package="uvdar_core",
            executable="led_manager_node",
            name="led_manager",
            namespace=namespace,
            output="screen",
            parameters=[{
                "uav_name": namespace,
                "sequence_file": sequence_file,
                "use_sim_time": use_sim_time,
            }],
            remappings=[
                ("~/baca_protocol_out", serial_send_topic),
            ],
        ),
    ])
