#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime


class MainStateRepeater(Node):
    """
    ROS2 node that manages the main state of the road surface repair machine.
    
    States: scanning, wheel_moving, filling, homing, wait_for_fill, cam_moving, 
            wait_for_fill_after_homing, free
    - Publishes to /main_state
    - Subscribes to /change_main_state, /current_xy_pos, /current_z_pos
    - Publishes warnings to /warning
    """
    
    def __init__(self):
        super().__init__('main_state_repeater')
        
        # Define valid states
        self.valid_states = {
            'scanning', 'wheel_moving', 'filling', 'homing', 
            'wait_for_fill', 'cam_moving', 'wait_for_fill_after_homing', 'free'
        }
        
        # Current state - start with homing
        self.current_state = 'homing'
        
        # Current robot position
        self.current_xy_pos = None  # [x, y]
        self.current_z_pos = None   # z
        
        # Home position - refers to stepper code where home is (0, 0) after homing
        # Z home is dummy value for now (no Z stepper code yet)
        self.home_xy = (0.0, 0.0)
        self.home_z = 0.0  # TODO: Update when Z stepper motor is implemented
        self.position_tolerance = 1.0  # Tolerance for home position check (steps)
        
        # Create publishers
        self.state_pub = self.create_publisher(
            String, '/main_state', 10
        )
        self.warning_pub = self.create_publisher(
            String, '/warning', 10
        )
        
        # Create subscriber for state change requests
        self.state_sub = self.create_subscription(
            String, '/change_main_state',
            self.state_change_callback, 10
        )
        
        # Create subscribers for position tracking
        self.xy_pos_sub = self.create_subscription(
            String, '/current_xy_pos',
            self.xy_pos_callback, 10
        )
        
        self.z_pos_sub = self.create_subscription(
            String, '/current_z_pos',
            self.z_pos_callback, 10
        )
        
        # Create a timer to periodically publish the current state
        self.timer = self.create_timer(0.1, self.publish_state)
        
        self.get_logger().info(
            f'MainStateRepeater initialized. Current state: {self.current_state}'
        )
    
    def xy_pos_callback(self, msg: String):
        """Handle current XY position updates."""
        try:
            # Parse the message - expecting format like "x,y"
            parts = msg.data.strip().split(',')
            if len(parts) == 2:
                self.current_xy_pos = (float(parts[0]), float(parts[1]))
        except (ValueError, IndexError):
            self.get_logger().warn(f"Invalid XY position format: {msg.data}")
    
    def z_pos_callback(self, msg: String):
        """Handle current Z position updates."""
        try:
            self.current_z_pos = float(msg.data.strip())
        except ValueError:
            self.get_logger().warn(f"Invalid Z position format: {msg.data}")
    
    def is_at_home_position(self) -> bool:
        """Check if robot is at home position."""
        if self.current_xy_pos is None or self.current_z_pos is None:
            return False
        
        xy_distance = (
            (self.current_xy_pos[0] - self.home_xy[0]) ** 2 +
            (self.current_xy_pos[1] - self.home_xy[1]) ** 2
        ) ** 0.5
        z_distance = abs(self.current_z_pos - self.home_z)
        
        return (xy_distance <= self.position_tolerance and 
                z_distance <= self.position_tolerance)
    
    def publish_state(self):
        """Periodically publish the current main state."""
        msg = String()
        msg.data = self.current_state
        self.state_pub.publish(msg)
    
    def publish_warning(self, warning_message: str):
        """Publish a warning message with timestamp to /warning topic."""
        msg = String()
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        msg.data = f"[{timestamp}] WARNING: {warning_message}"
        self.warning_pub.publish(msg)
        self.get_logger().warn(warning_message)
    
    def can_transition(self, new_state: str) -> bool:
        """
        Check if transition from current_state to new_state is allowed.
        
        Rules:
        1. If requesting "filling":
           - If not at home position: warn user that homing will be done first, deny
           - If at home position and in "wait_for_fill" or "wait_for_fill_after_homing": allow
           - Otherwise: deny with "Please do scanning first"
        2. If requesting "wait_for_fill_after_homing": allow (homing completion)
        3. If current state is not "free" and requesting other transitions: warn and deny
        4. If trying to change to "wheel_moving" or "cam_moving" from non-"free": deny
        
        Returns: (is_allowed, warning_message)
        """
        
        # Check if new state is valid
        if new_state not in self.valid_states:
            return False, f"Invalid state: {new_state}"
        
        # Check if already in that state
        if new_state == self.current_state:
            return False, f"Already in state: {new_state}"
        
        # Rule 1: Filling request with position checking
        if new_state == 'filling':
            # Check if not at home position
            if not self.is_at_home_position():
                return False, "Homing will be done first"
            # Can only fill from wait_for_fill or wait_for_fill_after_homing
            if self.current_state not in ['wait_for_fill', 'wait_for_fill_after_homing']:
                return False, "Please do scanning first"
            # At home position and in correct state, allow transition
            return True, ""
        
        # Rule 2: Allow transition to wait_for_fill_after_homing (homing completion - always allowed)
        if new_state == 'wait_for_fill_after_homing':
            return True, ""
        
        # Rule 3: If not in "free" state, deny most transitions
        if self.current_state != 'free':
            return False, f"{self.current_state} is going on, cannot change state"
        
        # Rule 4: "wheel_moving" and "cam_moving" can only start from "free"
        if new_state in ['wheel_moving', 'cam_moving'] and self.current_state != 'free':
            return False, f"{self.current_state} is going on, cannot change state"
        
        # Otherwise, transition is allowed
        return True, ""
    
    def state_change_callback(self, msg: String):
        """Handle incoming state change requests."""
        requested_state = msg.data.strip()
        
        # Check if transition is allowed
        is_allowed, warning = self.can_transition(requested_state)
        
        if is_allowed:
            # Update state
            previous_state = self.current_state
            self.current_state = requested_state
            self.get_logger().info(
                f'State changed from {previous_state} to {self.current_state}'
            )
        else:
            # Publish warning and deny state change
            self.publish_warning(warning)


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
