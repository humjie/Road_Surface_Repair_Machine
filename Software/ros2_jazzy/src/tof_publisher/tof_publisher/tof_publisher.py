"""
tof_publisher.py  —  VL53L1X → sensor_msgs/Range

CHANGES vs previous version:
  - Publishes EVERY reading as soon as it arrives (no rate gate).
  - Stale-hold feature kept but its window reduced to 0 s by default so it
    effectively does nothing unless the operator explicitly sets it > 0.
  - Serial readline timeout tightened to 0.02 s so the thread loops faster
    and misses fewer samples.
  - Buffer is cleared before each readline cycle so old bytes don't stall
    the parser.
  - Added a per-second throughput counter logged at INFO level.
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import Range


class TofPublisher(Node):
    def __init__(self):
        super().__init__('tof_publisher')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('serial_port',           '/dev/ttyUSB0')
        self.declare_parameter('baud_rate',             115200)
        self.declare_parameter('frame_id',              'tof_sensor_link')
        self.declare_parameter('min_range_m',           0.02)
        self.declare_parameter('max_range_m',           4.00)
        self.declare_parameter('fov_rad',               0.471)   # 27° VL53L1X
        # Stale-hold: republish the last valid sample for this many seconds
        # when the serial line goes quiet.  Set to 0.0 to disable entirely.
        self.declare_parameter('publish_last_if_stale', True)
        self.declare_parameter('last_sample_hold_s',    0.0)     # was 0.20

        self.serial_port          = self.get_parameter('serial_port').value
        self.baud_rate            = int(self.get_parameter('baud_rate').value)
        self.frame_id             = self.get_parameter('frame_id').value
        self.min_range_m          = float(self.get_parameter('min_range_m').value)
        self.max_range_m          = float(self.get_parameter('max_range_m').value)
        self.fov_rad              = float(self.get_parameter('fov_rad').value)
        self.publish_last_if_stale = bool(
            self.get_parameter('publish_last_if_stale').value)
        self.last_sample_hold_s   = float(
            self.get_parameter('last_sample_hold_s').value)

        # ── State ─────────────────────────────────────────────────────────────
        self._last_valid_range_m  : float | None = None
        self._last_valid_time_s   : float | None = None
        self._is_running          : bool  = True

        # Throughput counter
        self._pub_count : int   = 0
        self._rate_start: float = time.monotonic()

        # ── Serial ────────────────────────────────────────────────────────────
        self.ser: serial.Serial | None = None
        self._open_serial()

        # ── ROS publisher ─────────────────────────────────────────────────────
        self.publisher_ = self.create_publisher(Range, 'tof_data', 10)

        # ── Background reader thread ───────────────────────────────────────────
        self.read_thread = threading.Thread(
            target=self._serial_read_loop, daemon=True)
        self.read_thread.start()

        # ── Throughput logger (every 5 s) ─────────────────────────────────────
        self.create_timer(5.0, self._log_throughput)

        self.get_logger().info(
            'TofPublisher started (Threaded, max-rate)\n'
            f'  Port             : {self.serial_port} @ {self.baud_rate} baud\n'
            f'  Frame            : {self.frame_id}\n'
            f'  Range            : {self.min_range_m*1000:.0f}–'
            f'{self.max_range_m*1000:.0f} mm\n'
            f'  FOV              : {self.fov_rad:.3f} rad\n'
            f'  Stale-hold       : {self.last_sample_hold_s:.2f} s'
        )

    # ── Serial helpers ────────────────────────────────────────────────────────

    def _open_serial(self):
        try:
            if self.ser is not None and self.ser.is_open:
                self.ser.close()
            # Short timeout so the thread never blocks longer than 20 ms.
            self.ser = serial.Serial(
                self.serial_port, self.baud_rate, timeout=0.02)
            time.sleep(0.3)
            self.ser.reset_input_buffer()
            self.get_logger().info(f'Serial port {self.serial_port} opened.')
        except serial.SerialException as exc:
            self.ser = None
            self.get_logger().error(
                f'Failed to open {self.serial_port}: {exc}')

    # ── Main reader loop ──────────────────────────────────────────────────────

    def _serial_read_loop(self):
        """Dedicated thread: drain serial as fast as possible and publish every sample."""
        while rclpy.ok() and self._is_running:
            # ── (Re)connect if needed ─────────────────────────────────────────
            if self.ser is None or not self.ser.is_open:
                self._open_serial()
                if self.ser is None:
                    time.sleep(1.0)
                    continue

            try:
                raw = self.ser.readline().decode('utf-8', errors='ignore').strip()
                stamp = self.get_clock().now().to_msg()

                if not raw:
                    self._handle_stale_data()
                    continue

                # ── Parse ─────────────────────────────────────────────────────
                try:
                    distance_mm = float(raw)
                except ValueError:
                    self.get_logger().warn(f"Bad serial line: '{raw}'")
                    continue

                # ── Map to metres, using ±inf for out-of-range per REP-117 ────
                if distance_mm >= 8000.0:
                    distance_m = float('inf')
                elif distance_mm <= 0.0:
                    distance_m = float('-inf')
                else:
                    distance_m = distance_mm / 1000.0

                if distance_m < self.min_range_m:
                    distance_m = float('-inf')
                elif not math.isinf(distance_m) and distance_m > self.max_range_m:
                    distance_m = float('inf')

                # ── Publish immediately ───────────────────────────────────────
                self._publish_range(distance_m, stamp)

                # Cache only finite values for stale-hold
                if not math.isinf(distance_m):
                    self._last_valid_range_m = distance_m
                    self._last_valid_time_s  = self._now_sec()

            except (serial.SerialException, OSError) as exc:
                self.get_logger().error(f'Serial/OS error: {exc} — reconnecting.')
                self.ser = None

    # ── Stale-hold ────────────────────────────────────────────────────────────

    def _handle_stale_data(self):
        """Re-publish the last valid sample during a short serial dropout."""
        if not self.publish_last_if_stale:
            return
        if self.last_sample_hold_s <= 0.0:
            return
        if self._last_valid_range_m is None or self._last_valid_time_s is None:
            return
        age_s = self._now_sec() - self._last_valid_time_s
        if age_s <= self.last_sample_hold_s:
            self._publish_range(
                self._last_valid_range_m, self.get_clock().now().to_msg())

    # ── Publish helper ────────────────────────────────────────────────────────

    def _publish_range(self, distance_m: float, stamp):
        msg              = Range()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.radiation_type  = Range.INFRARED
        msg.field_of_view   = self.fov_rad
        msg.min_range       = self.min_range_m
        msg.max_range       = self.max_range_m
        msg.range           = distance_m
        self.publisher_.publish(msg)
        self._pub_count += 1

    # ── Throughput logger ─────────────────────────────────────────────────────

    def _log_throughput(self):
        elapsed = time.monotonic() - self._rate_start
        if elapsed > 0:
            rate = self._pub_count / elapsed
            self.get_logger().info(
                f'[ToF] Publishing at {rate:.1f} msg/s '
                f'({self._pub_count} total)')
        self._pub_count  = 0
        self._rate_start = time.monotonic()

    # ── Utilities ─────────────────────────────────────────────────────────────

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def destroy_node(self):
        self._is_running = False
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