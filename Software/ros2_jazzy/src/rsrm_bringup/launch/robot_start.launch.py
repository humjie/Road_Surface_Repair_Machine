from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. micro-ROS Agent for 4x4 DRIVE (ESP32)
    # Using /dev/ttyUSB0 as identified
    agent_drive = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='agent_drive',
        arguments=['serial', '--dev', '/dev/ttyUSB0', '-b', '115200'],
        output='screen',
        respawn=True
    )

    # 2. micro-ROS Agent for X-Y GANTRY (Arduino/ESP32)
    # Assuming the next one is /dev/ttyUSB1
    agent_gantry = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='agent_gantry',
        arguments=['serial', '--dev', '/dev/ttyUSB1', '-b', '115200'],
        output='screen',
        respawn=True
    )

    # 3. Foxglove Bridge (Corrected Executable Name)
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{'port': 8765}],
        output='screen'
    )

    return LaunchDescription([
        agent_drive,
        agent_gantry,
        foxglove_bridge
    ])