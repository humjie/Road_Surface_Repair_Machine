from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='foxglove_bridge', executable='foxglove_bridge', output='screen'),
        Node(package='custom_camera', executable='custom_camera', output='screen'),
        Node(package='tof_publisher', executable='tof_publisher', output='screen'),
        Node(package='tof_costmap', executable='tof_costmap_node', output='screen'),
    ])