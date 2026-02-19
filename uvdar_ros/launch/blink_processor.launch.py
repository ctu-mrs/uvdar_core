import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os


def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "uvdar_ros"
    pkg_share_path = get_package_share_directory(pkg_name)

    UAV_NAME = os.getenv('UAV_NAME', 'uav1')

    config_files = [pkg_share_path + '/config/config.yaml']

    namespace = UAV_NAME
    ld.add_action(
        ComposableNodeContainer(
            namespace='',
            name=namespace + '_uvdar_ros_blink',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package=pkg_name,
                    plugin='uvdar::blink_processor::BlinkProcessorComponent',
                    namespace=namespace,
                    name='BlinkProcessorComponent',
                    parameters=[
                        {
                            'config_files': config_files
                        },
                    ],
                    remappings=[
                        # ("~/lidar_in", "/velodyne_points"),
                    ],
                ),
            ],
            output='screen',
            # prefix=["gdbserver localhost:3000"],
        ))

    return ld
