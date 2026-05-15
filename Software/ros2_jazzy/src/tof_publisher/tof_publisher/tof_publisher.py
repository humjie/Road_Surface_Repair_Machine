"""
tof_publisher.py
━━━━━━━━━━━━━━━━
Reads VL53L1X distance readings from a serial-connected microcontroller
and republishes them as sensor_msgs/Range on /tof_data.

Fixes applied vs. previous version
──────────────────────────────────
  • Timer raised from 2 Hz → 50 Hz to match VL53L1X output rate
  • Removed ser.reset_input_buffer() which discarded data every tick
  • Drain ALL available lines per tick (don't leave queue building up)
  • Timestamp captured *before* readline() to reduce sync lag
  • Filter out-of-range / sentinel values (VL53L1X returns 8190 on no-target)
  • FOV corrected to 27° (0.471 rad) for VL53L1X default mode
  • Reconnect logic when serial port disappears
  • Use sensor-data QoS (best-effort, depth=1) for low-latency streaming
"""

import time

import rclpy
import serial
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Range


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


class TofPublisher(Node):
    def __init__(self):
        super().__init__('tof_publisher')

        self.declare_parameter('serial_port',   '/dev/ttyUSB0')
        self.declare_parameter('baud_rate',     115200)
        self.declare_parameter('poll_hz',       50.0)
        self.declare_parameter('frame_id',      'tof_sensor_link')  # no leading slash
        self.declare_parameter('min_range_m',   0.04)
        self.declare_parameter('max_range_m',   4.00)
        self.declare_parameter('fov_rad',       0.471)              # 27° VL53L1X

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate   = int(self.get_parameter('baud_rate').value)
        poll_hz          = float(self.get_parameter('poll_hz').value)
        self.frame_id    = self.get_parameter('frame_id').value
        self.min_range_m = float(self.get_parameter('min_range_m').value)
        self.max_range_m = float(self.get_parameter('max_range_m').value)
        self.fov_rad     = float(self.get_parameter('fov_rad').value)

        self.ser = None
        self._open_serial()

        self.publisher_ = self.create_publisher(Range, 'tof_data', SENSOR_QOS)

        timer_period = 1.0 / max(1.0, poll_hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'TofPublisher started\n'
            f'  Port      : {self.serial_port} @ {self.baud_rate} baud\n'
            f'  Poll rate : {poll_hz:.1f} Hz\n'
            f'  Frame     : {self.frame_id}\n'
            f'  Range     : {self.min_range_m*1000:.0f}–{self.max_range_m*1000:.0f} mm\n'
            f'  FOV       : {self.fov_rad:.3f} rad ({self.fov_rad*57.2958:.1f}°)'
        )

    # ─────────────────────────────────────────────────────────────────────────
    def _open_serial(self):
        """Open (or reopen) the serial port. Logs and sets self.ser=None on failure."""
        try:
            if self.ser is not None and self.ser.is_open:
                self.ser.close()
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.05)
            time.sleep(0.5)  # let device settle
            self.ser.reset_input_buffer()  # one-time flush of stale data on connect
            self.get_logger().info(f'Serial port {self.serial_port} opened.')
        except serial.SerialException as e:
            self.ser = None
            self.get_logger().error(f'Failed to open {self.serial_port}: {e}')

    # ─────────────────────────────────────────────────────────────────────────
    def timer_callback(self):
        if self.ser is None or not self.ser.is_open:
            self._open_serial()
            return

        try:
            # Drain ALL pending complete lines this tick.  Without this loop,
            # if the sensor publishes faster than we poll, the buffer fills up
            # and we'd be reading increasingly stale data.
            while self.ser.in_waiting > 0:
                stamp = self.get_clock().now().to_msg()      # capture BEFORE blocking read
                raw = self.ser.readline().decode('utf-8', errors='ignore').strip()

                if not raw:
                    continue

                try:
                    distance_mm = float(raw)
                except ValueError:
                    self.get_logger().warn(f"Bad serial line: '{raw}'")
                    continue

                # VL53L1X returns 8190 / 8191 when no target detected
                if distance_mm <= 0.0 or distance_mm >= 8000.0:
                    continue

                distance_m = distance_mm / 1000.0
                if distance_m < self.min_range_m or distance_m > self.max_range_m:
                    continue

                msg = Range()
                msg.header.stamp    = stamp
                msg.header.frame_id = self.frame_id
                msg.radiation_type  = Range.INFRARED
                msg.field_of_view   = self.fov_rad
                msg.min_range       = self.min_range_m
                msg.max_range       = self.max_range_m
                msg.range           = distance_m
                self.publisher_.publish(msg)

        except serial.SerialException as e:
            self.get_logger().error(f'Serial error: {e} — attempting reconnect.')
            self.ser = None
        except OSError as e:
            self.get_logger().error(f'OS error on serial port: {e} — reconnecting.')
            self.ser = None

    # ─────────────────────────────────────────────────────────────────────────
    def destroy_node(self):
        if self.ser is not None and self.ser.is_open:
            try:
                self.ser.close()
                self.get_logger().info('Serial port closed.')
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TofPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
