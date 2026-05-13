from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Consolidated micro-ROS agent handling all serial devices
    agent_multiserial = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='agent_multiserial',
        arguments=[
            'multiserial', 
            '--devs', '/dev/esp_wheel /dev/esp_stepper /dev/esp_zaxis /dev/esp_cam', 
            '-b', '115200'
        ],
        output='screen',
        respawn=True,
    )

    main_state_repeater = Node(
        package='main_state_repeater',
        executable='main_state_repeater',
        name='main_state_repeater',
        parameters=[{
            'initial_state': 'free',
            'main_state_topic': '/main_state',
            'change_main_state_topic': '/change_main_state',
        }],
        output='screen',
    )

    tof_costmap = Node(
        package='tof_costmap',
        executable='tof_costmap_node',
        name='tof_costmap_node',
        parameters=[{
            'input_topic': '/tof_data',
            'position_topic': '/current_xy_pos',
            'target_xy_topic': '/target_xy',
            'main_state_topic': '/main_state',
        }],
        output='screen',
    )

    tof_visualiser = Node(
        package='tof_visualiser',
        executable='tof_visualiser',
        name='tof_visualiser',
        parameters=[{
            'result_topic': '/tof_costmap',
        }],
        output='screen',
    )

    filling_control = Node(
        package='filling_control',
        executable='filling_control',
        name='filling_control',
        parameters=[{
            'result_topic': '/tof_result',
            'main_state_topic': '/main_state',
            'change_main_state_topic': '/change_main_state',
            'current_xy_topic': '/current_xy_pos',
            'current_z_topic': '/current_z_pos',
            'target_xy_topic': '/target_xy',
            'target_z_topic': '/target_z',
            'pump_cmd_topic': '/pump_cmd',
        }],
        output='screen',
    )

    custom_camera = Node(
        package='custom_camera',
        executable='custom_camera',
        name='custom_camera',
        output='screen',
    )

    return LaunchDescription([
        agent_multiserial,
        main_state_repeater,
        tof_costmap,
        tof_visualiser,
        filling_control,
        custom_camera,
    ])