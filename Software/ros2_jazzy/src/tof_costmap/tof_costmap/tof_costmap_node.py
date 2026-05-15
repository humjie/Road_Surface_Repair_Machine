"""
tof_costmap.py
━━━━━━━━━━━━━━
Builds a 2-D costmap of ToF readings during a scan and emits a JSON summary
on /tof_costmap when the scan completes.

Fixes applied vs. previous version
──────────────────────────────────
  • frame_id default changed from '/tof_sensor_link' → 'tof_sensor_link'
    (ROS 2 forbids leading slashes in TF frame IDs)
  • Baseline recalibration logic rewritten — bidirectional with deadband,
    only fires when readings are stable (low variance)
  • Removed dead-code guard in _publish_next_target() that compared the
    just-set target to itself
  • /change_main_state now publishes with TRANSIENT_LOCAL (latched) QoS so a
    state-manager subscriber that starts late still receives the 'free' msg
  • Target unit conversion uses explicit /1000.0 (not position_scale_m) to
    decouple from the position-message scale factor
  • Cell-update now averages multiple readings instead of taking only the
    first one — more robust against ToF noise
  • Marker publishing throttled to 10 Hz instead of every range callback
"""

import json
import statistics
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Range
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, String
from geometry_msgs.msg import Point, PointStamped
from std_srvs.srv import Trigger


LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class TofCostmapNode(Node):
    def __init__(self):
        super().__init__('tof_costmap_node')

        # ── Topics & frame ────────────────────────────────────────────────────
        self.declare_parameter('input_topic',        '/tof_data')
        self.declare_parameter('output_topic',       '/tof_costmap/markers')
        self.declare_parameter('ground_plane_topic', '/tof_costmap/ground_plane')
        self.declare_parameter('position_topic',     '/current_xy_pos')
        self.declare_parameter('target_xy_topic',    '/target_xy')
        self.declare_parameter('main_state_topic',   '/main_state')
        self.declare_parameter('frame_id',           'tof_sensor_link')

        # ── Scan geometry ─────────────────────────────────────────────────────
        self.declare_parameter('x_speed_mm_s',  10.0)
        self.declare_parameter('y_speed_mm_s',  10.0)
        self.declare_parameter('x_length_mm',  100.0)
        self.declare_parameter('y_length_mm',  100.0)
        self.declare_parameter('scan_step_mm',  10.0)

        # ── Hole detection ────────────────────────────────────────────────────
        self.declare_parameter('sensor_to_ground_m',      0.05)
        self.declare_parameter('hole_tolerance_m',        0.01)
        self.declare_parameter('hole_depth_range_m',      0.10)
        self.declare_parameter('ground_variance_range_m', 0.005)
        self.declare_parameter('calibration_window',      20)
        self.declare_parameter('recalibration_deadband_mm', 5.0)
        self.declare_parameter('recalibration_max_stdev_mm', 3.0)
        self.declare_parameter('sync_tolerance_s',        0.2)
        self.declare_parameter('position_scale_m',        0.001)
        self.declare_parameter('position_buffer_s',       2.0)

        # ── Read params ───────────────────────────────────────────────────────
        self.input_topic        = self.get_parameter('input_topic').value
        self.output_topic       = self.get_parameter('output_topic').value
        self.ground_plane_topic = self.get_parameter('ground_plane_topic').value
        self.position_topic     = self.get_parameter('position_topic').value
        self.target_xy_topic    = self.get_parameter('target_xy_topic').value
        self.main_state_topic   = self.get_parameter('main_state_topic').value
        self.frame_id           = self.get_parameter('frame_id').value

        self.x_speed_mm_s = float(self.get_parameter('x_speed_mm_s').value)
        self.y_speed_mm_s = float(self.get_parameter('y_speed_mm_s').value)
        self.x_length_mm  = float(self.get_parameter('x_length_mm').value)
        self.y_length_mm  = float(self.get_parameter('y_length_mm').value)
        self.scan_step_mm = float(self.get_parameter('scan_step_mm').value)

        self.sensor_to_ground_m         = float(self.get_parameter('sensor_to_ground_m').value)
        self.hole_tolerance_m           = float(self.get_parameter('hole_tolerance_m').value)
        self.hole_depth_range_m         = float(self.get_parameter('hole_depth_range_m').value)
        self.ground_variance_range_m    = float(self.get_parameter('ground_variance_range_m').value)
        self.calibration_window         = int(self.get_parameter('calibration_window').value)
        self.recalibration_deadband_m   = float(self.get_parameter('recalibration_deadband_mm').value) / 1000.0
        self.recalibration_max_stdev_m  = float(self.get_parameter('recalibration_max_stdev_mm').value) / 1000.0
        self.sync_tolerance_s           = float(self.get_parameter('sync_tolerance_s').value)
        self.position_scale_m           = float(self.get_parameter('position_scale_m').value)
        self.position_buffer_s          = float(self.get_parameter('position_buffer_s').value)

        self.current_baseline_m = self.sensor_to_ground_m
        self.cube_size_m        = self.scan_step_mm / 1000.0
        self.target_tolerance_m = max(self.cube_size_m * 0.25, 0.001)

        # ── State machine ─────────────────────────────────────────────────────
        self.scanning_active   = False
        self._result_published = False
        self.main_state        = 'free'
        self._scan_path        = []
        self._scan_index       = 0
        self._current_target   = None
        self._last_target_sent = None
        self.last_stamp_sec    = None

        # ── Data stores ───────────────────────────────────────────────────────
        # (grid_x, grid_y) → (x_m, y_m, distance_m, is_hole, sample_count)
        self.points_by_cell: dict = {}
        self.recent_readings: list = []
        self.position_buffer = deque()

        # ── ROS I/O ───────────────────────────────────────────────────────────
        # Use sensor-data QoS (best-effort) to match the publisher
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.subscription = self.create_subscription(
            Range, self.input_topic, self.range_callback, sensor_qos)

        self.position_sub = self.create_subscription(
            PointStamped, self.position_topic, self.position_callback, 10)

        self.main_state_sub = self.create_subscription(
            String, self.main_state_topic, self.main_state_callback, 10)

        # Latched so late-joining state-manager still gets the 'free' signal
        self.change_main_state_pub = self.create_publisher(
            String, '/change_main_state', LATCHED_QOS)

        self.target_pub  = self.create_publisher(Point,       self.target_xy_topic,    10)
        self.marker_pub  = self.create_publisher(MarkerArray, self.output_topic,        10)
        self.ground_pub  = self.create_publisher(MarkerArray, self.ground_plane_topic,  10)

        self.result_pub = self.create_publisher(String, '/tof_costmap', LATCHED_QOS)

        self.reset_srv = self.create_service(
            Trigger, 'tof_costmap/reset', self._reset_callback)

        # Scan-target tick at 20 Hz, marker publish at 10 Hz (was every range cb)
        self.timer = self.create_timer(0.05, self._scan_tick)
        self.marker_timer = self.create_timer(0.1, self._marker_tick)

        self.get_logger().info(
            f'TofCostmapNode started\n'
            f'  Input              : {self.input_topic}\n'
            f'  Costmap markers    : {self.output_topic}\n'
            f'  Ground plane       : {self.ground_plane_topic}\n'
            f'  Position topic     : {self.position_topic}\n'
            f'  Target XY topic    : {self.target_xy_topic}\n'
            f'  Main state         : {self.main_state_topic}\n'
            f'  Result topic       : /tof_costmap (latched JSON)\n'
            f'  Reset service      : tof_costmap/reset\n'
            f'  Frame              : {self.frame_id}\n'
            f'  Baseline (initial) : {self.sensor_to_ground_m:.4f} m\n'
            f'  Hole tolerance     : {self.hole_tolerance_m*1000:.1f} mm\n'
            f'  Recal deadband     : {self.recalibration_deadband_m*1000:.1f} mm\n'
            f'  Cube size          : {self.cube_size_m*1000:.1f} mm'
        )

    # ──────────────────────────────────────────────────────────────────────────
    def _reset_callback(self, request, response):
        self.points_by_cell.clear()
        self.recent_readings.clear()
        self.position_buffer.clear()
        self.current_baseline_m = self.sensor_to_ground_m
        self.scanning_active   = False
        self._result_published = False
        self._scan_path        = []
        self._scan_index       = 0
        self._current_target   = None
        self._last_target_sent = None
        response.success = True
        response.message = 'Scan reset — ready for a new sweep.'
        self.get_logger().info('Scan reset via service call.')
        return response

    # ──────────────────────────────────────────────────────────────────────────
    def position_callback(self, msg: PointStamped):
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        x_m = float(msg.point.x) * self.position_scale_m
        y_m = float(msg.point.y) * self.position_scale_m
        self.position_buffer.append((stamp_sec, x_m, y_m))
        self._prune_position_buffer(stamp_sec)

    # ──────────────────────────────────────────────────────────────────────────
    def main_state_callback(self, msg: String):
        state = msg.data.strip() or 'free'
        previous_state  = self.main_state
        self.main_state = state

        if state == 'scanning':
            if not self.scanning_active:
                self._start_scan()
            return

        if previous_state == 'scanning' and self.scanning_active:
            self.scanning_active = False
            if not self._result_published:
                self._publish_result_once()
                self._result_published = True

    # ──────────────────────────────────────────────────────────────────────────
    def _start_scan(self):
        self.points_by_cell.clear()
        self.recent_readings.clear()
        self.position_buffer.clear()
        self.current_baseline_m = self.sensor_to_ground_m
        self.scanning_active   = True
        self._result_published = False
        self._scan_path        = self._build_scan_path()
        self._scan_index       = 0
        self._current_target   = None
        self._last_target_sent = None
        self._publish_next_target()

    def _build_scan_path(self):
        path = []
        step = max(self.scan_step_mm, 1.0)
        x_positions = list(self._frange(0.0, self.x_length_mm, step))
        y_positions = list(self._frange(0.0, self.y_length_mm, step))
        forward = True
        for y_mm in y_positions:
            xs = x_positions if forward else list(reversed(x_positions))
            for x_mm in xs:
                path.append((x_mm, y_mm))
            forward = not forward
        return path

    @staticmethod
    def _frange(start_mm: float, stop_mm: float, step_mm: float):
        value = start_mm
        while value <= stop_mm + 1e-9:
            yield round(value, 6)
            value += step_mm

    def _publish_next_target(self):
        if not self.scanning_active:
            return

        if self._scan_index >= len(self._scan_path):
            if not self._result_published:
                self._publish_result_once()
                self._result_published = True
            self._publish_change_main_state('free')
            self.scanning_active = False
            return

        x_mm, y_mm = self._scan_path[self._scan_index]
        self._current_target = (x_mm, y_mm)

        # Don't re-publish the same waypoint
        if self._last_target_sent == self._current_target:
            return

        target = Point()
        target.x = float(x_mm)
        target.y = float(y_mm)
        target.z = 0.0
        self.target_pub.publish(target)
        self._last_target_sent = self._current_target

    def _publish_change_main_state(self, state: str):
        msg      = String()
        msg.data = state
        self.change_main_state_pub.publish(msg)

    def _latest_position(self):
        if not self.position_buffer:
            return None
        return self.position_buffer[-1][1], self.position_buffer[-1][2]

    def _target_reached(self):
        if self._current_target is None:
            return False
        current = self._latest_position()
        if current is None:
            return False
        x_m, y_m = current
        # Targets are unambiguously in mm — convert with /1000 explicitly
        tx_m = self._current_target[0] / 1000.0
        ty_m = self._current_target[1] / 1000.0
        return (abs(x_m - tx_m) <= self.target_tolerance_m and
                abs(y_m - ty_m) <= self.target_tolerance_m)

    def _scan_tick(self):
        if not self.scanning_active:
            return
        if self._current_target is None:
            self._publish_next_target()
            return
        if self._target_reached():
            self._scan_index += 1
            self._current_target   = None
            self._last_target_sent = None
            self._publish_next_target()

    def _prune_position_buffer(self, now_sec: float):
        while self.position_buffer and (now_sec - self.position_buffer[0][0]) > self.position_buffer_s:
            self.position_buffer.popleft()

    # ──────────────────────────────────────────────────────────────────────────
    def range_callback(self, msg: Range):
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        self.last_stamp_sec = stamp_sec

        distance_m = float(msg.range)
        self.maybe_recalibrate_baseline(distance_m)

        if self.scanning_active:
            pos = self._get_synced_position(stamp_sec)
            if pos is not None:
                x_m, y_m = pos
                self.update_costmap(distance_m, x_m, y_m)
        # NOTE: marker publishing moved to _marker_tick (10 Hz timer) to avoid
        # flooding Foxglove when the sensor publishes at 50 Hz.

    def _marker_tick(self):
        stamp = self.get_clock().now().to_msg()
        self.publish_markers(stamp)
        self.publish_ground_plane(stamp)

    def _get_synced_position(self, stamp_sec: float):
        if not self.position_buffer:
            return None
        best    = None
        best_dt = None
        for ts, x_m, y_m in self.position_buffer:
            dt = abs(stamp_sec - ts)
            if best_dt is None or dt < best_dt:
                best_dt = dt
                best    = (x_m, y_m)
        if best_dt is None or best_dt > self.sync_tolerance_s:
            return None
        return best

    # ──────────────────────────────────────────────────────────────────────────
    def maybe_recalibrate_baseline(self, distance_m: float):
        """
        Bidirectional recalibration with a stability check.

        Strategy:
          1. Keep a rolling window of recent readings.
          2. Once the window is full, check that readings are *stable*
             (stdev below threshold) — this means we're sitting on flat
             ground, not mid-pothole.
          3. If stable and the median differs from the current baseline by
             more than the deadband, accept it as the new baseline.
        """
        self.recent_readings.append(distance_m)
        if len(self.recent_readings) > self.calibration_window:
            self.recent_readings.pop(0)
        if len(self.recent_readings) < self.calibration_window:
            return

        try:
            window_stdev = statistics.stdev(self.recent_readings)
        except statistics.StatisticsError:
            return

        # Only recalibrate when the sensor is seeing a stable surface
        if window_stdev > self.recalibration_max_stdev_m:
            return

        candidate = statistics.median(self.recent_readings)
        delta_m   = candidate - self.current_baseline_m
        if abs(delta_m) < self.recalibration_deadband_m:
            return

        direction = 'UPWARD' if delta_m > 0 else 'DOWNWARD'
        self.get_logger().warn(
            f'Baseline recalibrated {direction} by {delta_m*1000:+.1f} mm: '
            f'{self.current_baseline_m:.4f} m → {candidate:.4f} m '
            f'(stdev={window_stdev*1000:.2f} mm)'
        )
        self.current_baseline_m = candidate
        self._recompute_hole_flags()

    def _recompute_hole_flags(self):
        threshold = self.current_baseline_m + self.hole_tolerance_m
        self.points_by_cell = {
            k: (x, y, z, z > threshold, n)
            for k, (x, y, z, _, n) in self.points_by_cell.items()
        }

    # ──────────────────────────────────────────────────────────────────────────
    def update_costmap(self, distance_m: float, x_m: float, y_m: float):
        """Accumulate readings per cell via running mean for noise robustness."""
        grid_x = int(round(x_m / self.cube_size_m))
        grid_y = int(round(y_m / self.cube_size_m))
        cell   = (grid_x, grid_y)

        z_new = max(0.0, distance_m)

        if cell in self.points_by_cell:
            old_x, old_y, old_z, _, n = self.points_by_cell[cell]
            n_new = n + 1
            z_avg = (old_z * n + z_new) / n_new
            # Position stays at first-seen value (the grid cell is fixed anyway)
            x_use, y_use = old_x, old_y
        else:
            n_new = 1
            z_avg = z_new
            x_use, y_use = x_m, y_m

        is_hole = z_avg > self.current_baseline_m + self.hole_tolerance_m
        self.points_by_cell[cell] = (x_use, y_use, z_avg, is_hole, n_new)

    # ──────────────────────────────────────────────────────────────────────────
    def _find_hole_clusters(self) -> list:
        hole_keys = {k for k, (_, _, _, h, _) in self.points_by_cell.items() if h}
        visited   = set()
        clusters  = []
        for start in hole_keys:
            if start in visited:
                continue
            keys  = []
            queue = [start]
            while queue:
                cell = queue.pop()
                if cell in visited or cell not in hole_keys:
                    continue
                visited.add(cell)
                keys.append(cell)
                gx, gy = cell
                queue.extend([(gx+1, gy), (gx-1, gy), (gx, gy+1), (gx, gy-1)])
            clusters.append([
                (self.points_by_cell[k][0],
                 self.points_by_cell[k][1],
                 self.points_by_cell[k][2]) for k in keys
            ])
        return clusters

    def _cluster_stats(self, cluster: list) -> tuple:
        """Returns (vol_m3, cx, cy, cz).  cz is at mid-depth, relative to floor (negative)."""
        area = self.cube_size_m ** 2
        tv = cx = cy = cz = 0.0
        for (x, y, d) in cluster:
            dep  = d - self.current_baseline_m
            if dep <= 0:
                continue
            vol  = area * dep
            tv  += vol
            cx  += x * vol
            cy  += y * vol
            cz  += -(dep / 2.0) * vol   # mid-depth below floor (floor = 0)
        if tv == 0.0:
            return 0.0, 0.0, 0.0, 0.0
        return tv, cx / tv, cy / tv, cz / tv

    # ──────────────────────────────────────────────────────────────────────────
    def _publish_result_once(self):
        clusters = self._find_hole_clusters()

        payload = {
            'baseline_m':  self.current_baseline_m,
            'scan_step_m': self.cube_size_m,
            'total_holes': len(clusters),
            'clusters':    []
        }

        for i, cluster in enumerate(clusters):
            vol_m3, cx, cy, cz = self._cluster_stats(cluster)
            payload['clusters'].append({
                'id':         i + 1,
                'cells':      len(cluster),
                'volume_cm3': round(vol_m3 * 1e6, 4),
                'centroid':   {'x': round(cx, 4), 'y': round(cy, 4), 'z': round(cz, 4)},
                'points': [
                    {'x': round(x, 4),
                     'y': round(y, 4),
                     'depth_m': round(d - self.current_baseline_m, 5)}
                    for (x, y, d) in cluster
                ]
            })

        msg      = String()
        msg.data = json.dumps(payload)
        self.result_pub.publish(msg)
        self._result_published = True

        lines = [
            '\n━━━ Scan Result ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━',
            f'  Baseline : {self.current_baseline_m:.4f} m',
            f'  Clusters : {len(clusters)}',
        ]
        for c in payload['clusters']:
            ct = c['centroid']
            lines.append(
                f"  #{c['id']:02d}  cells={c['cells']:4d}  "
                f"vol={c['volume_cm3']:8.3f} cm³  "
                f"centroid=({ct['x']:.4f}, {ct['y']:.4f}, {ct['z']:.4f}) m"
            )
        lines.append('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
        self.get_logger().info('\n'.join(lines))

    # ──────────────────────────────────────────────────────────────────────────
    def publish_ground_plane(self, stamp):
        if not self.points_by_cell:
            return
        ma    = MarkerArray()
        del_m = Marker()
        del_m.header.stamp    = stamp
        del_m.header.frame_id = self.frame_id
        del_m.ns              = 'ground_plane'
        del_m.id              = 0
        del_m.action          = Marker.DELETEALL
        ma.markers.append(del_m)

        m                    = Marker()
        m.header.stamp       = stamp
        m.header.frame_id    = self.frame_id
        m.ns                 = 'ground_plane'
        m.id                 = 1
        m.type               = Marker.CUBE
        m.action             = Marker.ADD
        m.pose.orientation.w = 1.0
        m.lifetime.sec       = 0
        m.pose.position.x    = (self.x_length_mm / 2.0) / 1000.0
        m.pose.position.y    = (self.y_length_mm / 2.0) / 1000.0
        m.pose.position.z    = 0.0
        m.scale.x            = self.x_length_mm / 1000.0
        m.scale.y            = self.y_length_mm / 1000.0
        m.scale.z            = 0.001
        m.color              = ColorRGBA(r=0.95, g=0.95, b=0.95, a=0.20)
        ma.markers.append(m)
        self.ground_pub.publish(ma)

    # ──────────────────────────────────────────────────────────────────────────
    @staticmethod
    def _lerp3(t: float, c0: tuple, c1: tuple) -> tuple:
        return tuple(c0[i] + (c1[i] - c0[i]) * t for i in range(3))

    def _ground_color(self, dist_m: float) -> ColorRGBA:
        dev = dist_m - self.current_baseline_m
        vr  = max(self.ground_variance_range_m, 1e-6)
        t   = max(0.0, min(1.0, (dev / vr + 1.0) / 2.0))
        if t < 0.5:
            r, g, b = self._lerp3(t * 2.0,
                                  (0.88, 0.95, 1.00),
                                  (0.25, 0.50, 0.82))
        else:
            r, g, b = self._lerp3((t - 0.5) * 2.0,
                                  (0.25, 0.50, 0.82),
                                  (0.80, 0.65, 0.20))
        return ColorRGBA(r=r, g=g, b=b, a=0.65)

    def _hole_color(self, depth_m: float) -> ColorRGBA:
        dr = max(self.hole_depth_range_m, 1e-6)
        t  = min(1.0, depth_m / dr)
        if t < 0.5:
            r, g, b = self._lerp3(t * 2.0,
                                  (1.00, 0.85, 0.05),
                                  (1.00, 0.28, 0.05))
        else:
            r, g, b = self._lerp3((t - 0.5) * 2.0,
                                  (1.00, 0.28, 0.05),
                                  (0.50, 0.02, 0.02))
        return ColorRGBA(r=r, g=g, b=b, a=0.78 + 0.20 * t)

    # ──────────────────────────────────────────────────────────────────────────
    def publish_markers(self, stamp):
        clusters     = self._find_hole_clusters()
        marker_array = MarkerArray()

        # DELETEALL + ADDs in the SAME message → no flicker
        del_m = Marker()
        del_m.header.stamp    = stamp
        del_m.header.frame_id = self.frame_id
        del_m.ns              = 'tof_costmap'
        del_m.id              = 0
        del_m.action          = Marker.DELETEALL
        marker_array.markers.append(del_m)

        half    = self.cube_size_m / 2.0
        next_id = 1

        for (_gx, _gy), (x_m, y_m, z_m, is_hole, _n) in self.points_by_cell.items():
            m                    = Marker()
            m.header.stamp       = stamp
            m.header.frame_id    = self.frame_id
            m.ns                 = 'tof_costmap'
            m.id                 = next_id
            m.type               = Marker.CUBE
            m.action             = Marker.ADD
            m.pose.orientation.w = 1.0
            m.lifetime.sec       = 0
            m.scale.x            = self.cube_size_m
            m.scale.y            = self.cube_size_m
            m.pose.position.x    = x_m
            m.pose.position.y    = y_m
            next_id             += 1

            if is_hole:
                depth_m           = z_m - self.current_baseline_m
                height            = max(depth_m, self.cube_size_m)
                m.scale.z         = height
                m.pose.position.z = -(height / 2.0)
                m.color           = self._hole_color(depth_m)
            else:
                m.scale.z         = self.cube_size_m
                m.pose.position.z = half
                m.color           = self._ground_color(z_m)

            marker_array.markers.append(m)

        for i, cluster in enumerate(clusters):
            vol_m3, cx, cy, cz = self._cluster_stats(cluster)
            vol_cm3             = vol_m3 * 1e6

            t                    = Marker()
            t.header.stamp       = stamp
            t.header.frame_id    = self.frame_id
            t.ns                 = 'tof_costmap'
            t.id                 = next_id
            t.type               = Marker.TEXT_VIEW_FACING
            t.action             = Marker.ADD
            t.pose.orientation.w = 1.0
            t.lifetime.sec       = 0
            next_id             += 1

            t.pose.position.x = cx
            t.pose.position.y = cy
            t.pose.position.z = cz + 0.03
            t.scale.z         = self.cube_size_m * 2.5
            t.color           = ColorRGBA(r=1.0, g=1.0, b=0.2, a=1.0)
            t.text            = f'#{i+1}  {vol_cm3:.1f} cm³\n{len(cluster)} cells'
            marker_array.markers.append(t)

        self.marker_pub.publish(marker_array)


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
