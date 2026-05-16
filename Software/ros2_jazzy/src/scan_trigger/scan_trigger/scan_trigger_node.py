#!/usr/bin/env python3
"""
scan_trigger_node.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
ROS 2 Jazzy node that:
  - Subscribes to /start_scan  (std_msgs/Bool)
  - When it receives True, writes 'S' over USB serial to the ESP32

Usage:
  1. Copy this file to your ROS 2 workspace, e.g.:
       ~/ros2_ws/src/scan_trigger/scan_trigger/scan_trigger_node.py

  2. Install pyserial on the Pi if not already present:
       pip3 install pyserial

  3. Find your ESP32 serial port:
       ls /dev/ttyUSB* /dev/ttyACM*
     It is usually /dev/ttyUSB0 or /dev/ttyACM0.
     Update SERIAL_PORT below if needed.

  4. Build and run:
       cd ~/ros2_ws
       colcon build --packages-select scan_trigger
       source install/setup.bash
       ros2 run scan_trigger scan_trigger_node

  5. In Foxglove, publish True on /start_scan to trigger the scan.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import serial
import serial.serialutil

# ── CONFIG ──────────────────────────────────────────────────────────
SERIAL_PORT = '/dev/esp_stepper'   # ← change if your ESP32 appears elsewhere
BAUD_RATE   = 115200
TOPIC       = '/start_scan'
# ────────────────────────────────────────────────────────────────────


class ScanTriggerNode(Node):

    def __init__(self):
        super().__init__('scan_trigger_node')

        # Open serial port
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            self.get_logger().info(
                f'Serial port opened: {SERIAL_PORT} @ {BAUD_RATE} baud')
        except serial.serialutil.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise

        # Subscribe to /start_scan
        self.sub = self.create_subscription(
            Bool,
            TOPIC,
            self.start_scan_callback,
            10
        )
        self.get_logger().info(f'Subscribed to {TOPIC}. Ready.')

    def start_scan_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info(
                'Received True on /start_scan — sending S to ESP32')
            try:
                self.ser.write(b'S')
            except serial.serialutil.SerialException as e:
                self.get_logger().error(f'Serial write failed: {e}')
        else:
            self.get_logger().info(
                'Received False on /start_scan — ignored')

    def destroy_node(self):
        if self.ser.is_open:
            self.ser.close()
            self.get_logger().info('Serial port closed.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ScanTriggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()