from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the full Road Surface Repair Machine pipeline.

    Nodes
    -----
    micro_ros_agent   : bridges the ESP32 stepper firmware over serial
    tof_publisher     : reads VL53L1X over serial, publishes /tof_data
    tof_one_shot_run  : waits for homing, scans, publishes /tof_costmap, fills holes
    hole_sim          : (optional) simulated scan + Foxglove 3-D markers —
                        comment out when running with real hardware
    """

    # ── Micro-ROS agent (ESP32 stepper firmware) ──────────────────────────────
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', '/dev/esp_stepper', '-b', '115200'],
    )

    # ── ToF sensor publisher ───────────────────────────────────────────────────
    tof_pub = Node(
        package='tof_one_shot_run',
        executable='tof_publisher',
        name='tof_publisher',
        output='screen',
        parameters=[
            {'serial_port':           '/dev/ttyUSB0'},
            {'baud_rate':             115200},
            {'frame_id':              'tof_sensor_link'},
            {'min_range_m':           0.02},
            {'max_range_m':           4.00},
            {'fov_rad':               0.471},
            {'publish_last_if_stale': True},
            {'last_sample_hold_s':    0.0},   # 0 = publish every reading immediately
        ],
    )

    # ── One-shot scan + fill orchestrator ─────────────────────────────────────
    oneshot = Node(
        package='tof_one_shot_run',
        executable='tof_one_shot_run',
        name='tof_one_shot_run',
        output='screen',
        parameters=[
            {'input_topic':          '/tof_data'},
            {'current_xy_topic':     '/current_xy_pos'},
            {'target_xy_topic':      '/target_xy'},
            {'pump_cmd_topic':       '/pump_cmd'},
            {'result_topic':         '/tof_costmap'},
            {'frame_id':             'tof_sensor_link'},
            # Geometry
            {'sensor_to_ground_m':   0.03},
            {'hole_tolerance_m':     0.005},
            # Scan window — must stay within ±1800 (firmware travel limit)
            {'scan_x_start_steps':   -1600.0},
            {'scan_x_end_steps':      1600.0},
            {'scan_y_start_steps':   -1600.0},
            {'scan_y_end_steps':      1600.0},
            {'scan_step_units':       400.0},
            {'steps_per_mm':          160.0},
            # Timing
            {'sample_window_s':       0.25},
            {'move_timeout_s':        20.0},
            {'home_timeout_s':        45.0},
            {'fill_settle_s':         0.35},
            # Pump
            {'pump_rate_cm3_s':       1.0},
            {'min_pump_ms':           200},
            {'max_pump_ms':           60000},
        ],
    )

    # ── Simulated scan + Foxglove markers (bench-testing only) ────────────────
    # Comment this node out when running with real hardware.
    hole_sim = Node(
        package='tof_one_shot_run',
        executable='tof_one_shot_run',
        name='tof_one_shot_run',
        output='screen',
        parameters=[
            {'frame_id':             'tof_sensor_link'},
            {'scan_x_start_steps':   -1600.0},
            {'scan_x_end_steps':      1600.0},
            {'scan_y_start_steps':   -1600.0},
            {'scan_y_end_steps':      1600.0},
            {'scan_step_units':       400.0},
            {'steps_per_mm':          160.0},
            {'sensor_to_ground_m':    0.03},
            {'hole_tolerance_m':      0.005},
            {'start_x_steps':         0.0},
            {'start_y_steps':         0.0},
            {'pre_move_x_offset':    -500.0},
            {'pre_move_y_offset':    -500.0},
            {'sim_speed_steps_s':     2000.0},
            {'sim_dwell_s':           0.05},
        ],
    )

    # ── Real hardware launch ───────────────────────────────────────────────────
    # return LaunchDescription([micro_ros_agent, tof_pub, oneshot])

    # ── Simulation launch (no hardware required) ───────────────────────────────
    return LaunchDescription([hole_sim])