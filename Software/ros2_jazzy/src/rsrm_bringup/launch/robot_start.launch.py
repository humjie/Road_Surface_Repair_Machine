from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    agent_wheel = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='agent_wheel',
        arguments=['serial', '--dev', '/dev/esp_wheel', '-b', '115200'],
        output='screen',
        respawn=True,
    )

    agent_stepper = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='agent_stepper',
        arguments=['serial', '--dev', '/dev/esp_stepper', '-b', '115200'],
        output='screen',
        respawn=True,
    )

    agent_zaxis = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='agent_zaxis',
        arguments=['serial', '--dev', '/dev/esp_zaxis', '-b', '115200'],
        output='screen',
        respawn=True,
    )

    agent_pump_cam = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='agent_pump_cam',
        arguments=['serial', '--dev', '/dev/esp_pump_cam', '-b', '115200'],
        output='screen',
        respawn=True,
    )

    main_state_repeater = Node(
        package='main_state_repeater',
        executable='main_state_repeater',
        name='main_state_repeater',
        output='screen',
    )

    tof_costmap = Node(
        package='tof_costmap',
        executable='tof_costmap_node',
        name='tof_costmap_node',
        output='screen',
    )

    tof_visualiser = Node(
        package='tof_visualiser',
        executable='tof_visualiser',
        name='tof_visualiser',
        output='screen',
    )

    filling_control = Node(
        package='filling_control',
        executable='filling_control',
        name='filling_control',
        output='screen',
    )

    custom_camera = Node(
        package='custom_camera',
        executable='custom_camera',
        name='custom_camera',
        output='screen',
    )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{'port': 8765}],
        output='screen',
    )

    return LaunchDescription([
        agent_wheel,
        agent_stepper,
        agent_zaxis,
        agent_pump_cam,
        main_state_repeater,
        tof_costmap,
        tof_visualiser,
        filling_control,
        custom_camera,
        foxglove_bridge,
    ])