import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import statistics


class TofCostmapNode(Node):
    def __init__(self):
        super().__init__('tof_costmap_node')

        # ── Topics & frame ────────────────────────────────────────────────────
        self.declare_parameter('input_topic', 'tof_data')
        self.declare_parameter('output_topic', 'tof_costmap/markers')
        self.declare_parameter('frame_id', 'tof_sensor_link')

        # ── Scan geometry ─────────────────────────────────────────────────────
        self.declare_parameter('x_speed_mm_s', 10.0)
        self.declare_parameter('y_speed_mm_s', 10.0)
        self.declare_parameter('x_length_mm', 100.0)
        self.declare_parameter('y_length_mm', 100.0)
        self.declare_parameter('scan_step_mm', 10.0)

        # ── Hole detection ────────────────────────────────────────────────────
        # sensor_to_ground_m: user-defined nominal distance from sensor to flat
        # ground in metres.  The sensor faces DOWNWARD so Z+ in the sensor frame
        # points toward the ground.  A reading GREATER than this value means the
        # sensor is looking into a void → hole.
        self.declare_parameter('sensor_to_ground_m', 0.05)

        # hole_tolerance_m: dead-band above the baseline before flagging a hole.
        # Keeps sensor noise from generating false positives.
        self.declare_parameter('hole_tolerance_m', 0.01)

        # calibration_window: number of consecutive readings in the sliding
        # window used for auto-calibration.
        self.declare_parameter('calibration_window', 20)

        # recalibration_threshold: fraction of window readings that must be
        # LESS THAN the current baseline before we conclude the sensor has
        # physically moved closer to the ground and shift the baseline UPWARD.
        self.declare_parameter('recalibration_threshold', 0.8)

        # ── Read params ───────────────────────────────────────────────────────
        self.input_topic  = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.frame_id     = self.get_parameter('frame_id').value

        self.x_speed_mm_s = float(self.get_parameter('x_speed_mm_s').value)
        self.y_speed_mm_s = float(self.get_parameter('y_speed_mm_s').value)
        self.x_length_mm  = float(self.get_parameter('x_length_mm').value)
        self.y_length_mm  = float(self.get_parameter('y_length_mm').value)
        self.scan_step_mm = float(self.get_parameter('scan_step_mm').value)

        self.sensor_to_ground_m      = float(self.get_parameter('sensor_to_ground_m').value)
        self.hole_tolerance_m        = float(self.get_parameter('hole_tolerance_m').value)
        self.calibration_window      = int(self.get_parameter('calibration_window').value)
        self.recalibration_threshold = float(self.get_parameter('recalibration_threshold').value)

        # Effective baseline — starts at user value and can only go UP.
        self.current_baseline_m = self.sensor_to_ground_m

        # ── State machine ─────────────────────────────────────────────────────
        self.SCANNING_X = 'SCANNING_X'
        self.STEPPING_Y = 'STEPPING_Y'
        self.DONE       = 'DONE'

        self.state          = self.SCANNING_X
        self.current_x_mm   = 0.0
        self.current_y_mm   = 0.0
        self.direction_x    = 1
        self.target_y_mm    = 0.0
        self.last_grid_x    = -1
        self.last_stamp_sec = None

        # ── Data stores ───────────────────────────────────────────────────────
        # points_by_cell: (grid_x, grid_y) → (x_m, y_m, distance_m, is_hole)
        self.points_by_cell: dict = {}
        # Sliding window of recent readings for auto-calibration
        self.recent_readings: list = []

        # cube_size_m: side length of every marker cube = scan step.
        # Cubes tile seamlessly over the scanned surface.
        self.cube_size_m = self.scan_step_mm / 1000.0

        # ── ROS I/O ───────────────────────────────────────────────────────────
        self.subscription = self.create_subscription(
            Range, self.input_topic, self.range_callback, 10)
        self.publisher = self.create_publisher(MarkerArray, self.output_topic, 10)

        self.get_logger().info(
            f'TofCostmapNode started\n'
            f'  Input  : {self.input_topic}\n'
            f'  Output : {self.output_topic}  (visualization_msgs/MarkerArray)\n'
            f'  Nominal sensor-to-ground : {self.sensor_to_ground_m:.3f} m\n'
            f'  Hole tolerance           : {self.hole_tolerance_m:.3f} m\n'
            f'  Cube size                : {self.cube_size_m * 1000:.1f} mm'
        )

    # ──────────────────────────────────────────────────────────────────────────
    # ROS callback
    # ──────────────────────────────────────────────────────────────────────────
    def range_callback(self, msg: Range):
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if self.last_stamp_sec is None:
            self.last_stamp_sec = stamp_sec
            dt = 0.0
        else:
            dt = max(0.0, stamp_sec - self.last_stamp_sec)
            self.last_stamp_sec = stamp_sec

        distance_m = float(msg.range)

        # Recalibrate from every reading, even after the scan finishes, so the
        # final published markers stay correct if the sensor height drifts.
        self.maybe_recalibrate_baseline(distance_m)

        if self.state != self.DONE:
            self.update_scan_state(dt)
            self.update_costmap(distance_m)

        # Always republish so Foxglove always has a fresh MarkerArray.
        self.publish_markers(msg.header.stamp)

    # ──────────────────────────────────────────────────────────────────────────
    # Baseline auto-calibration — UPWARD ONLY
    # ──────────────────────────────────────────────────────────────────────────
    def maybe_recalibrate_baseline(self, distance_m: float):
        """
        Physical scenario this handles
        ──────────────────────────────
        User sets sensor_to_ground_m = 0.50 m, but the robot sags and the
        sensor ends up only 0.45 m above the floor.  Nearly every reading
        comes in around 0.45 m, which is below the 0.50 m baseline.  The
        hole-detection threshold (0.50 + tolerance) would be too far away
        from actual floor readings — real holes may still be caught but the
        baseline window gives a false sense of a healthy scan.

        Correction
        ──────────
        When >= recalibration_threshold of the window is BELOW the current
        baseline, set baseline = median(window) — but ONLY if that candidate
        is HIGHER than the current baseline (upward shift only).

        Why only upward?
        ────────────────
        Real holes produce readings ABOVE the baseline, so they cannot
        contribute to the "below baseline" count that triggers recalibration.
        The upward-only guard therefore has no practical effect on legitimate
        hole scans while preventing pathological downward drift.
        """
        self.recent_readings.append(distance_m)
        if len(self.recent_readings) > self.calibration_window:
            self.recent_readings.pop(0)

        if len(self.recent_readings) < self.calibration_window:
            return  # not enough data yet

        n_below = sum(1 for r in self.recent_readings if r < self.current_baseline_m)
        fraction_below = n_below / self.calibration_window

        if fraction_below < self.recalibration_threshold:
            return  # readings look healthy, no adjustment needed

        candidate = statistics.median(self.recent_readings)

        # Only accept an upward correction.
        if candidate <= self.current_baseline_m:
            return

        delta_mm = (candidate - self.current_baseline_m) * 1000.0
        if delta_mm < 5.0:      # ignore sub-5 mm jitter
            return

        self.get_logger().warn(
            f'Baseline recalibrated UPWARD by {delta_mm:.1f} mm: '
            f'{self.current_baseline_m:.4f} m → {candidate:.4f} m  '
            f'({fraction_below * 100:.0f}% of last {self.calibration_window} '
            f'readings were below the old baseline)'
        )
        self.current_baseline_m = candidate
        self._recompute_hole_flags()

    def _recompute_hole_flags(self):
        """Re-evaluate every stored cell against the updated baseline."""
        threshold = self.current_baseline_m + self.hole_tolerance_m
        self.points_by_cell = {
            key: (x, y, z, z > threshold)
            for key, (x, y, z, _) in self.points_by_cell.items()
        }

    # ──────────────────────────────────────────────────────────────────────────
    # Scan state machine
    # ──────────────────────────────────────────────────────────────────────────
    def update_scan_state(self, dt: float):
        if self.state == self.SCANNING_X:
            self.current_x_mm += self.x_speed_mm_s * dt * self.direction_x

            at_right = self.direction_x == 1  and self.current_x_mm >= self.x_length_mm
            at_left  = self.direction_x == -1 and self.current_x_mm <= 0.0

            if at_right or at_left:
                self.current_x_mm = self.x_length_mm if at_right else 0.0
                self.target_y_mm  = self.current_y_mm + self.scan_step_mm
                self.direction_x *= -1
                self.state = self.STEPPING_Y

        elif self.state == self.STEPPING_Y:
            self.current_y_mm += self.y_speed_mm_s * dt

            if self.current_y_mm >= self.target_y_mm:
                self.current_y_mm = self.target_y_mm

                if self.current_y_mm > self.y_length_mm:
                    self.state = self.DONE
                    self.get_logger().info(
                        'ToF scan complete — keeping final costmap published.')
                    self._report_hole_analysis()
                else:
                    self.last_grid_x = -1
                    self.state = self.SCANNING_X

    # ──────────────────────────────────────────────────────────────────────────
    # Costmap update
    # ──────────────────────────────────────────────────────────────────────────
    def update_costmap(self, distance_m: float):
        if self.state != self.SCANNING_X:
            return

        current_grid_x = int(round(self.current_x_mm / self.scan_step_mm))
        if current_grid_x == self.last_grid_x:
            return  # same grid column as last reading — skip duplicate

        grid_y = int(round(self.current_y_mm / self.scan_step_mm))
        x_m    = self.current_x_mm / 1000.0
        y_m    = self.current_y_mm / 1000.0
        z_m    = max(0.0, distance_m)

        is_hole = z_m > self.current_baseline_m + self.hole_tolerance_m

        self.points_by_cell[(current_grid_x, grid_y)] = (x_m, y_m, z_m, is_hole)
        self.last_grid_x = current_grid_x

    # ──────────────────────────────────────────────────────────────────────────
    # Hole volume & centroid report (called once at end of scan)
    # ──────────────────────────────────────────────────────────────────────────
    def _report_hole_analysis(self):
        """
        Volume
        ──────
        Each hole cell is a vertical column whose cross-section is
        cube_size × cube_size and whose depth is (distance − baseline).
        Summing all column volumes gives the total void volume.

        Centroid
        ────────
        Volume-weighted mean of each column's 3-D centre of mass.
        Column centre Z = baseline + depth/2  (sensor frame, Z+ downward).
        """
        hole_cells = [
            (x, y, z) for (x, y, z, is_hole) in self.points_by_cell.values()
            if is_hole
        ]

        if not hole_cells:
            self.get_logger().info('Hole analysis: no hole detected in scanned area.')
            return

        cell_area_m2 = self.cube_size_m ** 2
        total_vol_m3 = 0.0
        cx = cy = cz = 0.0

        for (x_m, y_m, z_m) in hole_cells:
            depth_m  = z_m - self.current_baseline_m
            col_vol  = cell_area_m2 * depth_m
            total_vol_m3 += col_vol
            cx += x_m * col_vol
            cy += y_m * col_vol
            cz += (self.current_baseline_m + depth_m / 2.0) * col_vol

        centroid_x = cx / total_vol_m3
        centroid_y = cy / total_vol_m3
        centroid_z = cz / total_vol_m3

        self.get_logger().info(
            f'\n'
            f'━━━ Hole Analysis ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n'
            f'  Hole cells   : {len(hole_cells)}\n'
            f'  Volume       : {total_vol_m3:.6f} m³  '
            f'({total_vol_m3 * 1e6:.2f} cm³)\n'
            f'  Centroid (m) : x={centroid_x:.4f}  y={centroid_y:.4f}  '
            f'z={centroid_z:.4f}\n'
            f'  Baseline     : {self.current_baseline_m:.4f} m\n'
            f'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
        )

    # ──────────────────────────────────────────────────────────────────────────
    # MarkerArray publisher  (Foxglove-compatible CUBE markers)
    # ──────────────────────────────────────────────────────────────────────────
    def publish_markers(self, stamp):
        """
        Coordinate convention — downward-facing sensor, Z+ toward ground
        ─────────────────────────────────────────────────────────────────
        Ground cell (flat surface):
          scale.z  = cube_size_m
          centre Z = baseline + cube_size_m / 2
          → top face of cube sits flush with the ground plane

        Hole cell (void below ground):
          scale.z  = depth_m  (= distance_m − baseline_m), min 1 cube step
          centre Z = baseline + depth_m / 2
          → cube fills the void from the ground surface downward
        """
        marker_array = MarkerArray()

        # DELETEALL clears markers that no longer exist (e.g. after recalibration
        # flips some cells from hole→ground or ground→hole).
        delete_all = Marker()
        delete_all.header.stamp    = stamp
        delete_all.header.frame_id = self.frame_id
        delete_all.ns              = 'tof_costmap'
        delete_all.id              = 0
        delete_all.action          = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        half = self.cube_size_m / 2.0

        for idx, ((_gx, _gy), (x_m, y_m, z_m, is_hole)) in \
                enumerate(self.points_by_cell.items()):

            m                    = Marker()
            m.header.stamp       = stamp
            m.header.frame_id    = self.frame_id
            m.ns                 = 'tof_costmap'
            m.id                 = idx + 1   # 0 reserved for DELETEALL
            m.type               = Marker.CUBE
            m.action             = Marker.ADD
            m.pose.orientation.w = 1.0
            m.lifetime.sec       = 0         # persist until next update

            m.scale.x = self.cube_size_m
            m.scale.y = self.cube_size_m

            if is_hole:
                depth_m  = z_m - self.current_baseline_m
                height   = max(depth_m, self.cube_size_m)  # at least 1 step tall
                m.scale.z            = height
                m.pose.position.z    = self.current_baseline_m + height / 2.0
                # Red, brighter as the hole gets deeper (saturates at 200 mm)
                intensity = min(1.0, depth_m / 0.2)
                m.color = ColorRGBA(
                    r=0.8 + 0.2 * intensity, g=0.1, b=0.1, a=0.9)
            else:
                m.scale.z            = self.cube_size_m
                m.pose.position.z    = self.current_baseline_m + half
                # Semi-transparent blue-grey for normal ground
                m.color = ColorRGBA(r=0.3, g=0.5, b=0.8, a=0.45)

            m.pose.position.x = x_m
            m.pose.position.y = y_m
            marker_array.markers.append(m)

        self.publisher.publish(marker_array)


# ──────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = TofCostmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ToF costmap node interrupted, shutting down.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()