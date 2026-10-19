import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    camera_share = get_package_share_directory('hik_camera')
    driver_share = get_package_share_directory('livox_ros2_driver')
    projection_share = get_package_share_directory('livox_pcd_projection')

    camera_launch = os.path.join(camera_share, 'launch', 'hik_camera.launch.py')
    driver_config = os.path.join(driver_share, 'config', 'livox_lidar_config.json')
    projection_config = os.path.join(projection_share, 'config', 'project.yaml')
    foxglove_url = PythonExpression([
        "'foxglove://open?ds=foxglove-websocket&ds.url=ws://localhost:' + '",
        LaunchConfiguration('foxglove_port'),
        "' + '&ds.name=Hikvision-Livox'",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'my_color', default_value='2',
            description='0=red, 1=blue, 2=keep both colors'),
        DeclareLaunchArgument(
            'publish_debug', default_value='true',
            description='publish armor_debug image'),
        DeclareLaunchArgument(
            'projection_debug', default_value='true',
            description='publish debug_image with projected LiDAR points'),
        DeclareLaunchArgument(
            'foxglove_port', default_value='8765',
            description='Foxglove WebSocket bridge port'),
        DeclareLaunchArgument(
            'open_foxglove', default_value='true',
            description='open Foxglove Studio and connect to the bridge'),

        # Use the reliable image QoS expected by the detector and projector.
        # The sensor-data profile is best-effort and is incompatible with
        # message_filters' default reliable subscription.
        Node(
            package='hik_camera',
            executable='hik_camera_node',
            name='hik_camera',
            output='screen',
            emulate_tty=True,
            parameters=[
                os.path.join(camera_share, 'config', 'camera_params.yaml'),
                {
                    'camera_info_url': 'package://hik_camera/config/camera_info.yaml',
                    'use_sensor_data_qos': False,
                },
            ],
        ),

        # xfer_format=1 is required by livox_pcd_projection: CustomMsg.
        # An empty command-line code leaves device selection to the JSON,
        # where the actual Mid-70 broadcast code is configured.
        Node(
            package='livox_ros2_driver',
            executable='livox_ros2_driver_node',
            name='livox_lidar_publisher',
            output='screen',
            parameters=[
                {
                    'xfer_format': 1,
                    'multi_topic': 0,
                    'data_src': 0,
                    'publish_freq': 10.0,
                    'output_data_type': 0,
                    'frame_id': 'livox_frame',
                    'user_config_path': driver_config,
                    'cmdline_input_bd_code': '',
                },
            ],
        ),

        Node(
            package='ros2_armor_detector',
            executable='armor_detector_node',
            name='armor_detector',
            output='screen',
            parameters=[
                {
                    'input_topic': '/image_raw',
                    'detection_topic': '/armor_detections',
                    'debug_topic': '/armor_debug',
                    'my_color': LaunchConfiguration('my_color'),
                    'publish_debug': LaunchConfiguration('publish_debug'),
                    'model_config': os.path.join(
                        get_package_share_directory('ros2_armor_detector'),
                        'config', 'armor_v8_cpu.json'),
                    'model_root': '/home/szx/works/hik_livox_workspace/models/openvino',
                },
            ],
        ),

        Node(
            package='hik_livox_system',
            executable='stamper_cpp',
            name='stamper',
            output='screen',
            parameters=[projection_config],
        ),
        Node(
            package='hik_livox_system',
            executable='projector_cpp',
            name='projector',
            output='screen',
            parameters=[
                projection_config,
                {
                    'debug': LaunchConfiguration('projection_debug'),
                    'max_data_age': 0.5,
                    'debug_max_points': 5000,
                    'debug_scale': 0.5,
                    'debug_rate': 10.0,
                },
            ],
        ),

        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[
                {
                    'port': LaunchConfiguration('foxglove_port'),
                    'address': '127.0.0.1',
                },
            ],
        ),

        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=['foxglove-studio', foxglove_url],
                    name='foxglove_studio',
                    output='screen',
                    condition=IfCondition(LaunchConfiguration('open_foxglove')),
                ),
            ],
        ),
    ])
