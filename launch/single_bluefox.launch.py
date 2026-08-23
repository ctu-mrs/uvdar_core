#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, TextSubstitution, PathJoinSubstitution
from launch_ros.actions import Node, LoadComposableNodes
from launch.substitutions import PythonExpression, IfElseSubstitution
from launch.actions import LogInfo
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from launch_ros.substitutions import FindPackageShare
import subprocess

def get_available_cameras():
    # Query the driver utility so launch diagnostics can list connected cameras.
    try:
        result = subprocess.run(['ros2', 'run', 'bluefox2', 'bluefox2_list_cameras'], 
                              capture_output=True, text=True)
        return result.stdout
    except:
        return []

def generate_launch_description():
    declare_use_camera_name = DeclareLaunchArgument(
        'use_camera_name',
        default_value=EnvironmentVariable('USE_CAMERA_NAME', default_value="false"),
        description='Defines whether the node should use camera name with serial number in it. It may be not very practical when only one camera is used. User has to know camera serial number to use proper namespace when definning parameters.'
    )
    
    declare_custom_config = DeclareLaunchArgument(
        'custom_config',
        default_value='',
        description='config from the user'
    )
    
    # Declare launch arguments
    declare_node_start_delay = DeclareLaunchArgument(
        'node_start_delay',
        default_value='0',
        description='Node delay for multiple cameras (driver can crash if run multiple times in the same moment)'
    )
        
    # Devices listing utility returns camera serial numbers divided by space. The last character is newline, so it is thrown away with '-1' indexing.
    devices = get_available_cameras().split(" ")[0:-1]
    selected_device = ''
    if len(devices):
        selected_device = devices[0]
        devices_search_log = LogInfo(msg=f"Found Bluefox2 devices: {devices}. If user does not select particular device, the device with serial number {selected_device} will be used.")
    else:
        devices_search_log = LogInfo(msg="No Bluefox2 devices found.")
         
    declare_device = DeclareLaunchArgument(
        'device',
        default_value=selected_device,
        description='Device serial number (can be found by running bluefox2_list_cameras)'
    )
    
    declare_uav_name = DeclareLaunchArgument(
        'uav_name',
        default_value=EnvironmentVariable('UAV_NAME'),
        description='Camera namespace (used for node name and topic namespace)'
    )
    
    declare_camera_name = DeclareLaunchArgument(
        'camera_name',
        default_value='',
        description='Camera name (used for node name and topic namespace)'
    )
        
    
    # Camera settings
    declare_fps = DeclareLaunchArgument('fps', default_value='60', description='Frame rate')
    declare_aec = DeclareLaunchArgument('aec', default_value='false', description='Auto exposure control')
    declare_des_grey_value = DeclareLaunchArgument('des_grey_value', default_value='128', description='Desired brightness 0-255 (only when aec == true)')
    declare_expose_upper_limit_us = DeclareLaunchArgument('expose_upper_limit_us', default_value='100000', description='Upper limit of exposure time (only when aec == true)')
    declare_max_expose_jump = DeclareLaunchArgument('max_expose_jump', default_value='1000000', description='Maximal change of exposure time in one step (only when aec == true)')
    declare_acs = DeclareLaunchArgument('acs', default_value='2', description='Auto exposure control speed (0 - slow, 1 - medium, 2 - fast)')
    declare_expose_us = DeclareLaunchArgument('expose_us', default_value='2000', description='Exposure time in microseconds (only when aec == false)')
    declare_agc = DeclareLaunchArgument('agc', default_value='false', description='Auto gain control')
    declare_gain_db = DeclareLaunchArgument('gain_db', default_value='0.0', description='Gain (only when agc == false)')
    declare_wbp = DeclareLaunchArgument('wbp', default_value='-1', description='White balance parameter')
    declare_idpf = DeclareLaunchArgument('idpf', default_value='0', description='Pixel format')
    declare_mm = DeclareLaunchArgument('mm', default_value='0', description='Mirror the captured image')
    declare_cbm = DeclareLaunchArgument('cbm', default_value='0', description='Camera binning mode')
    declare_ctm = DeclareLaunchArgument('ctm', default_value='1', description='Camera trigger mode')
    declare_dcfm = DeclareLaunchArgument('dcfm', default_value='0', description='Dark current filter')
    declare_hdr = DeclareLaunchArgument('hdr', default_value='false', description='High dynamic range')
    declare_request = DeclareLaunchArgument('request', default_value='0', description='Request capture queue count')
    
    # Compression settings
    declare_compressed_jpeg_quality = DeclareLaunchArgument('compressed_jpeg_quality', default_value='90')
    declare_theora_keyframe_frequency = DeclareLaunchArgument('theora_keyframe_frequency', default_value='60')
    declare_theora_target_bitrate = DeclareLaunchArgument('theora_target_bitrate', default_value='50000')
    declare_theora_quality = DeclareLaunchArgument('theora_quality', default_value='8')
    declare_theora_optimize_for = DeclareLaunchArgument('theora_optimize_for', default_value='0')
    
    # Node settings
    declare_output = DeclareLaunchArgument('output', default_value='screen', description='Text output to screen/log')
    declare_rectify = DeclareLaunchArgument('rectify', default_value='false', description='Run rectification')
    declare_view = DeclareLaunchArgument('view', default_value='false', description='Run camera viewer')

    declare_image = DeclareLaunchArgument('image', default_value='image_raw', description='Image topic for viewer')

    # Add camera_name below uav_name when multiple cameras share one vehicle.
    # Relative remappings then remain isolated without embedding namespaces in
    # every topic string.
    camera_ns = IfElseSubstitution(
        condition=PythonExpression(['"', LaunchConfiguration('camera_name'), '" != ""']),
        if_value=PathJoinSubstitution([LaunchConfiguration('uav_name'), LaunchConfiguration('camera_name')]),
        else_value=LaunchConfiguration('uav_name'),
    )

    # Environment setup for custom libusb
    env_vars = {
        'LD_LIBRARY_PATH': '/opt/mvIMPACT_acquire_libusb:' + os.environ.get('LD_LIBRARY_PATH', '')
    }
    
    def get_processed_launch_objects(context):
        _custom_config_file = LaunchConfiguration('custom_config').perform(context)
        
        objects = [
            LogInfo(msg=f"custom config file: {_custom_config_file}"),
        ]
        
        camera_name = LaunchConfiguration('camera_name').perform(context)

        frame_id = EnvironmentVariable('UAV_NAME').perform(context) + '/bluefox'


        if camera_name != '':
           frame_id += '_' + camera_name 

        print( "Selected Device ID: ", LaunchConfiguration('device').perform(context))

        parameters = [{
            'identifier': LaunchConfiguration('device').perform(context),
            'frame_id': frame_id,
            'camera_name': camera_name,
            'fps': LaunchConfiguration('fps'),
            'idpf': LaunchConfiguration('idpf'),
            'aec': LaunchConfiguration('aec'),
            'expose_us': LaunchConfiguration('expose_us'),
            'agc': LaunchConfiguration('agc'),  # Fixed the typo from 'aec'
            'gain_db': LaunchConfiguration('gain_db'),
            'cbm': LaunchConfiguration('cbm'),
            'ctm': LaunchConfiguration('ctm'),
            'dcfm': LaunchConfiguration('dcfm'),
            'hdr': LaunchConfiguration('hdr'),
            'wbp': LaunchConfiguration('wbp'),
            'request': LaunchConfiguration('request'),
            'mm': LaunchConfiguration('mm'),
            'expose_upper_limit_us': LaunchConfiguration('expose_upper_limit_us'),
            'max_expose_jump': LaunchConfiguration('max_expose_jump'),
            'des_grey_value': LaunchConfiguration('des_grey_value'),
            'acs': LaunchConfiguration('acs'),
            'image_raw/compressed/jpeg_quality': LaunchConfiguration('compressed_jpeg_quality'),
            'image_raw/theora/keyframe_frequency': LaunchConfiguration('theora_keyframe_frequency'),
            'image_raw/theora/target_bitrate': LaunchConfiguration('theora_target_bitrate'),
            'image_raw/theora/quality': LaunchConfiguration('theora_quality'),
            'image_raw/theora/optimize_for': LaunchConfiguration('theora_optimize_for'),
        }]
        
        if _custom_config_file != '':
            print("appending params")
            parameters.append(_custom_config_file)
            
        objects.append(DeclareLaunchArgument(name='container_id', default_value=''))
        objects.append(DeclareLaunchArgument(name='standalone', default_value='true'))
        
        camera_node = ComposableNode(
            package='bluefox2',
            plugin='bluefox2::BluefoxSingleComponent',  # Assuming the nodelet is converted to a regular node
            name=['bluefox_', camera_name] if camera_name != '' else "bluefox",
            namespace=camera_ns,
            parameters=parameters,
            extra_arguments=[{'use_intra_process_comms': True}],
            remappings=[
                ('expose_us', 'bluefox/expose_us'),
                ('gain_db', 'bluefox/gain_db'),
            ],
        )

        # Relative to camera_ns, so this subscribes to exactly what
        # camera_node published (<camera_ns>/image_raw, <camera_ns>/camera_info)
        # without needing to know uav_name/camera_name itself.
        rectify_remappings=[
            ('image', 'image_raw'),
            ('camera_info', 'camera_info'),
        ]

        rectify_node = ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_mono',
            namespace=camera_ns,
            condition=IfCondition(LaunchConfiguration('rectify')),
            remappings=rectify_remappings
        )
        
        objects.append(
            LoadComposableNodes(
                condition=UnlessCondition(LaunchConfiguration('standalone')),
                composable_node_descriptions=[camera_node, rectify_node],
                target_container=LaunchConfiguration('container_id'),
            )
        )
        
        objects.append(
            ComposableNodeContainer(
                condition=IfCondition(LaunchConfiguration('standalone')),
                name=['bluefox2_container_', camera_name] if camera_name != '' else 'bluefox2_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                output=LaunchConfiguration('output'),
                respawn=False,
                additional_env=env_vars,
                #prefix='xterm -e gdb -ex run --args',
                composable_node_descriptions=[camera_node, rectify_node]
            )
        )
        
        return objects
    
    bluefox2_node = OpaqueFunction(function=get_processed_launch_objects)
    
    # Camera viewer node
    viewer_node = Node(
        package='image_view',
        executable='image_view',
        name='viewer',
        namespace=camera_ns,
        condition=IfCondition(LaunchConfiguration('view')),
        output=LaunchConfiguration('output'),
        arguments=[PythonExpression(['image:=', LaunchConfiguration('image')])]
    )
    
    
    return LaunchDescription([
        # Declare all arguments
        declare_use_camera_name,
        devices_search_log,
        declare_custom_config,
        declare_node_start_delay,
        declare_device,
        declare_uav_name,
        declare_camera_name,
        declare_fps,
        declare_aec,
        declare_des_grey_value,
        declare_expose_upper_limit_us,
        declare_max_expose_jump,
        declare_acs,
        declare_expose_us,
        declare_agc,
        declare_gain_db,
        declare_wbp,
        declare_idpf,
        declare_mm,
        declare_cbm,
        declare_ctm,
        declare_dcfm,
        declare_hdr,
        declare_request,
        declare_compressed_jpeg_quality,
        declare_theora_keyframe_frequency,
        declare_theora_target_bitrate,
        declare_theora_quality,
        declare_theora_optimize_for,
        declare_output,
        declare_rectify,
        declare_view,
        declare_image,
        
        # Launch nodes
        bluefox2_node,
        viewer_node,
    ])
