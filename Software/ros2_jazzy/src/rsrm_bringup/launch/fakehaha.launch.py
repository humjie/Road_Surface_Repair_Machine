from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    custom_camera = Node(
        package='custom_camera',
        executable='custom_camera',
        name='custom_camera',
        output='screen',
    )

    scan_trigger = Node(
        package='scan_trigger',
        executable='scan_trigger_node',
        name='scan_trigger_node',
        output='screen',
    )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': 8765,
            'topic_whitelist': ['/camera/image_raw'],  # ← replace with your actual topic
        }],
        output='screen',
    )

    return LaunchDescription([
        custom_camera,
        scan_trigger,
        foxglove_bridge,
    ])