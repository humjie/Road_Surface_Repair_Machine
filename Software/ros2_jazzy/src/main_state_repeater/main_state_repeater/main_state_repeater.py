#!/usr/bin/env python3
"""
main_state_repeater.py
━━━━━━━━━━━━━━━━━━━━━━
Central state machine for the road-surface-repair system.

Transition rules (operator-gated wait_for_fill with abort/rescan)
─────────────────────────────────────────────────────────────────
  free          -> homing, scanning, wheel_moving, cam_moving
  homing        -> free
  scanning      -> wait_for_fill                (set by tof_costmap when scan ends)
  wait_for_fill -> filling   (operator presses 'Start Filling')
                -> free      (operator presses 'Abort')
                -> scanning  (operator presses 'Rescan')
  filling       -> free
  wheel_moving  -> free
  cam_moving    -> free
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
)
from std_msgs.msg import String


LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


# Operator-gated flow with optional abort/rescan:
#   free -> scanning -> wait_for_fill -> { filling | free (abort) | scanning (rescan) }
LEGAL_TRANSITIONS = {
    'free':          {'homing', 'scanning', 'wheel_moving', 'cam_moving'},
    'homing':        {'free'},
    'scanning':      {'wait_for_fill'},
    'wait_for_fill': {'filling', 'free', 'scanning'},   # filling=proceed, free=abort, scanning=rescan
    'filling':       {'free'},
    'wheel_moving':  {'free'},
    'cam_moving':    {'free'},
}

VALID_STATES = set(LEGAL_TRANSITIONS.keys())


class MainStateRepeater(Node):
    def __init__(self):
        super().__init__('main_state_repeater')

        self.declare_parameter('initial_state',           'free')
        self.declare_parameter('main_state_topic',        '/main_state')
        self.declare_parameter('change_main_state_topic', '/change_main_state')
        self.declare_parameter('heartbeat_topic',         '/main_state_heartbeat')
        self.declare_parameter('heartbeat_hz',            1.0)

        self.current_state = (
            str(self.get_parameter('initial_state').value).strip() or 'free'
        )
        if self.current_state not in VALID_STATES:
            self.get_logger().warn(
                f"Initial state '{self.current_state}' not in valid set "
                f"{sorted(VALID_STATES)} — defaulting to 'free'."
            )
            self.current_state = 'free'

        self.main_state_topic        = self.get_parameter('main_state_topic').value
        self.change_main_state_topic = self.get_parameter('change_main_state_topic').value
        heartbeat_topic              = self.get_parameter('heartbeat_topic').value
        heartbeat_hz                 = float(self.get_parameter('heartbeat_hz').value)

        self.state_pub = self.create_publisher(
            String, self.main_state_topic, LATCHED_QOS)

        self.heartbeat_pub = self.create_publisher(
            String, heartbeat_topic, 10)

        self.state_sub = self.create_subscription(
            String,
            self.change_main_state_topic,
            self.state_change_callback,
            10,
        )

        period = 1.0 / max(0.1, heartbeat_hz)
        self.timer = self.create_timer(period, self.publish_heartbeat)

        self._publish_state()

        rules = '\n'.join(
            f'    {src:14s} -> {sorted(dsts)}'
            for src, dsts in LEGAL_TRANSITIONS.items()
        )
        self.get_logger().info(
            f"MainStateRepeater initialized\n"
            f"  Initial state    : {self.current_state}\n"
            f"  /main_state QoS  : latched (TRANSIENT_LOCAL)\n"
            f"  Heartbeat        : {heartbeat_topic} @ {heartbeat_hz:.1f} Hz\n"
            f"  Legal transitions:\n{rules}"
        )

    def _publish_state(self):
        msg = String()
        msg.data = self.current_state
        self.state_pub.publish(msg)

    def publish_heartbeat(self):
        msg = String()
        msg.data = f'alive:{self.current_state}'
        self.heartbeat_pub.publish(msg)

    def state_change_callback(self, msg: String):
        requested = msg.data.strip()
        if not requested:
            return

        if requested == self.current_state:
            return

        if requested not in VALID_STATES:
            self.get_logger().warn(
                f"Ignoring unknown state '{requested}' "
                f"(valid: {sorted(VALID_STATES)})"
            )
            return

        allowed = LEGAL_TRANSITIONS.get(self.current_state, set())
        if requested not in allowed:
            self.get_logger().warn(
                f"Rejecting illegal transition "
                f"'{self.current_state}' -> '{requested}' "
                f"(allowed from {self.current_state}: {sorted(allowed)})"
            )
            return

        previous = self.current_state
        self.current_state = requested
        self._publish_state()
        self.get_logger().info(f"State: {previous} -> {self.current_state}")


def main(args=None):
    rclpy.init(args=args)
    node = MainStateRepeater()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()