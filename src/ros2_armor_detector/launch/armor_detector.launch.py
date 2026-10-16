from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory('ros2_armor_detector')
    return LaunchDescription([
        DeclareLaunchArgument('input_topic', default_value='/image_raw'),
        DeclareLaunchArgument('detection_topic', default_value='/armor_detections'),
        DeclareLaunchArgument('debug_topic', default_value='/armor_debug'),
        DeclareLaunchArgument('my_color', default_value='2'),
        Node(
            package='ros2_armor_detector',
            executable='armor_detector_node',
            name='armor_detector',
            output='screen',
            parameters=[{
                'input_topic': LaunchConfiguration('input_topic'),
                'detection_topic': LaunchConfiguration('detection_topic'),
                'debug_topic': LaunchConfiguration('debug_topic'),
                'my_color': LaunchConfiguration('my_color'),
                'model_config': os.path.join(package_share, 'config', 'armor_v8_cpu.json'),
                'model_root': '/home/szx/works/hik_livox_workspace/models/openvino',
            }],
        ),
    ])
