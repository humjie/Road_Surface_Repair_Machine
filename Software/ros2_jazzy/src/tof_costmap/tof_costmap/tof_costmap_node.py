import json
import statistics
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Range
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, String
from geometry_msgs.msg import PointStamped
from std_srvs.srv import Trigger

# TRANSIENT_LOCAL = "latched" in ROS 2.
# The result message is published once; this QoS ensures the visualiser
# receives it even if it starts after the scan completes.
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
        self.declare_parameter('input_topic',        'tof_data')
        self.declare_parameter('output_topic',       'tof_costmap/markers')
        self.declare_parameter('ground_plane_topic', 'tof_costmap/ground_plane')
        self.declare_parameter('position_topic',     'current_pos')
        self.declare_parameter('main_cmd_topic',     'main_cmd')
        self.declare_parameter('stepper_state_topic','stepper_state')
        self.declare_parameter('main_state_topic',   'main_state')
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
        self.declare_parameter('recalibration_threshold',  0.8)
        self.declare_parameter('sync_tolerance_s',        0.2)
        self.declare_parameter('position_scale_m',        0.001)
        self.declare_parameter('position_buffer_s',       2.0)

        # ── Read params ───────────────────────────────────────────────────────
        self.input_topic        = self.get_parameter('input_topic').value
        self.output_topic       = self.get_parameter('output_topic').value
        self.ground_plane_topic = self.get_parameter('ground_plane_topic').value
        self.position_topic     = self.get_parameter('position_topic').value
        self.main_cmd_topic     = self.get_parameter('main_cmd_topic').value
        self.stepper_state_topic = self.get_parameter('stepper_state_topic').value
        self.main_state_topic   = self.get_parameter('main_state_topic').value
        self.frame_id           = self.get_parameter('frame_id').value

        self.x_speed_mm_s = float(self.get_parameter('x_speed_mm_s').value)
        self.y_speed_mm_s = float(self.get_parameter('y_speed_mm_s').value)
        self.x_length_mm  = float(self.get_parameter('x_length_mm').value)
        self.y_length_mm  = float(self.get_parameter('y_length_mm').value)
        self.scan_step_mm = float(self.get_parameter('scan_step_mm').value)

        self.sensor_to_ground_m      = float(self.get_parameter('sensor_to_ground_m').value)
        self.hole_tolerance_m        = float(self.get_parameter('hole_tolerance_m').value)
        self.hole_depth_range_m      = float(self.get_parameter('hole_depth_range_m').value)
        self.ground_variance_range_m = float(self.get_parameter('ground_variance_range_m').value)
        self.calibration_window      = int(self.get_parameter('calibration_window').value)
        self.recalibration_threshold = float(self.get_parameter('recalibration_threshold').value)
        self.sync_tolerance_s        = float(self.get_parameter('sync_tolerance_s').value)
        self.position_scale_m        = float(self.get_parameter('position_scale_m').value)
        self.position_buffer_s       = float(self.get_parameter('position_buffer_s').value)

        self.current_baseline_m = self.sensor_to_ground_m
        self.cube_size_m        = self.scan_step_mm / 1000.0

        # ── State machine ─────────────────────────────────────────────────────
        self.SCANNING_X = 'SCANNING_X'
        self.STEPPING_Y = 'STEPPING_Y'
        self.DONE       = 'DONE'
        self._init_scan_state()

        self.scanning_active = False
        self._result_published = False
        self.stepper_state = 'available'
        self._last_main_state = None

        # ── Data stores ───────────────────────────────────────────────────────
        # (grid_x, grid_y) → (x_m, y_m, distance_m, is_hole)
        self.points_by_cell: dict = {}
        self.recent_readings: list = []
        self.position_buffer = deque()
        self.last_grid_cell = None

        # ── ROS I/O ───────────────────────────────────────────────────────────
        self.subscription = self.create_subscription(
            Range, self.input_topic, self.range_callback, 10)

        self.position_sub = self.create_subscription(
            PointStamped, self.position_topic, self.position_callback, 10)

        self.main_cmd_sub = self.create_subscription(
            String, self.main_cmd_topic, self.main_cmd_callback, 10)

        self.stepper_state_sub = self.create_subscription(
            String, self.stepper_state_topic, self.stepper_state_callback, 10)

        self.marker_pub  = self.create_publisher(MarkerArray, self.output_topic,       10)
        self.ground_pub  = self.create_publisher(MarkerArray, self.ground_plane_topic, 10)
        self.main_state_pub = self.create_publisher(String, self.main_state_topic, 10)

        # Result published ONCE at scan end as a JSON string.
        # Schema: { baseline_m, clusters: [ {id, cells, volume_cm3,
        #           centroid:{x,y,z}, points:[{x,y,depth_m},...] }, ... ] }
        self.result_pub = self.create_publisher(String, 'tof_costmap/result', LATCHED_QOS)  # latched: visualiser receives even if late

        self.reset_srv = self.create_service(
            Trigger, 'tof_costmap/reset', self._reset_callback)

        self.get_logger().info(
            f'TofCostmapNode started\n'
            f'  Input              : {self.input_topic}\n'
            f'  Costmap markers    : {self.output_topic}\n'
            f'  Ground plane       : {self.ground_plane_topic}\n'
            f'  Position topic     : {self.position_topic}\n'
            f'  Main cmd           : {self.main_cmd_topic}\n'
            f'  Stepper state      : {self.stepper_state_topic}\n'
            f'  Main state         : {self.main_state_topic}\n'
            f'  Result (once/scan) : tof_costmap/result  (JSON String)\n'
            f'  Reset service      : tof_costmap/reset\n'
            f'  Baseline           : {self.sensor_to_ground_m:.4f} m\n'
            f'  Hole tolerance     : {self.hole_tolerance_m:.4f} m\n'
            f'  Hole depth range   : 0–{self.hole_depth_range_m*1000:.1f} mm\n'
            f'  Ground var range   : ±{self.ground_variance_range_m*1000:.1f} mm\n'
            f'  Cube size          : {self.cube_size_m*1000:.1f} mm'
        )

    # ──────────────────────────────────────────────────────────────────────────
    def _init_scan_state(self):
        self.state          = self.SCANNING_X
        self.current_x_mm   = 0.0
        self.current_y_mm   = 0.0
        self.direction_x    = 1
        self.target_y_mm    = 0.0
        self.last_grid_cell = None
        self.last_stamp_sec = None

    def _reset_callback(self, request, response):
        self._init_scan_state()
        self.points_by_cell.clear()
        self.recent_readings.clear()
        self.position_buffer.clear()
        self.last_grid_cell = None
        self.current_baseline_m = self.sensor_to_ground_m
        self.scanning_active = False
        self._result_published = False
        response.success = True
        response.message = 'Scan reset — ready for a new sweep.'
        self.get_logger().info('Scan reset via service call.')
        self._publish_main_state()
        return response

    def position_callback(self, msg: PointStamped):
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        x_m = float(msg.point.x) * self.position_scale_m
        y_m = float(msg.point.y) * self.position_scale_m
        self.position_buffer.append((stamp_sec, x_m, y_m))
        self._prune_position_buffer(stamp_sec)

    def main_cmd_callback(self, msg: String):
        cmd = msg.data.strip()
        if cmd in ('START_SCAN', 'SCAN'):
            self._start_scan()
        elif cmd == 'RESET':
            self._init_scan_state()
            self.points_by_cell.clear()
            self.recent_readings.clear()
            self.position_buffer.clear()
            self.last_grid_cell = None
            self.current_baseline_m = self.sensor_to_ground_m
            self.scanning_active = False
            self._result_published = False
            self._publish_main_state()

    def stepper_state_callback(self, msg: String):
        self.stepper_state = msg.data.strip() or 'available'
        if self.scanning_active and self.stepper_state == 'available':
            self.scanning_active = False
            if not self._result_published:
                self._publish_result_once()
                self._result_published = True
        self._publish_main_state()

    def _start_scan(self):
        self._init_scan_state()
        self.points_by_cell.clear()
        self.recent_readings.clear()
        self.position_buffer.clear()
        self.last_grid_cell = None
        self.current_baseline_m = self.sensor_to_ground_m
        self.scanning_active = True
        self._result_published = False
        self._publish_main_state()

    def _prune_position_buffer(self, now_sec: float):
        while self.position_buffer and (now_sec - self.position_buffer[0][0]) > self.position_buffer_s:
            self.position_buffer.popleft()

    def _combined_main_state(self) -> str:
        if self.scanning_active:
            return 'scanning'
        if self.stepper_state:
            return self.stepper_state
        return 'available'

    def _publish_main_state(self):
        state = self._combined_main_state()
        if state == self._last_main_state:
            return
        msg = String()
        msg.data = state
        self.main_state_pub.publish(msg)
        self._last_main_state = state

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

        self.publish_markers(msg.header.stamp)
        self.publish_ground_plane(msg.header.stamp)

    def _get_synced_position(self, stamp_sec: float):
        if not self.position_buffer:
            return None
        best = None
        best_dt = None
        for ts, x_m, y_m in self.position_buffer:
            dt = abs(stamp_sec - ts)
            if best_dt is None or dt < best_dt:
                best_dt = dt
                best = (x_m, y_m)
        if best_dt is None or best_dt > self.sync_tolerance_s:
            return None
        return best

    # ──────────────────────────────────────────────────────────────────────────
    def maybe_recalibrate_baseline(self, distance_m: float):
        self.recent_readings.append(distance_m)
        if len(self.recent_readings) > self.calibration_window:
            self.recent_readings.pop(0)
        if len(self.recent_readings) < self.calibration_window:
            return

        n_below        = sum(1 for r in self.recent_readings if r < self.current_baseline_m)
        fraction_below = n_below / self.calibration_window
        if fraction_below < self.recalibration_threshold:
            return

        candidate = statistics.median(self.recent_readings)
        if candidate <= self.current_baseline_m:
            return
        delta_mm = (candidate - self.current_baseline_m) * 1000.0
        if delta_mm < 5.0:
            return

        self.get_logger().warn(
            f'Baseline recalibrated UPWARD by {delta_mm:.1f} mm: '
            f'{self.current_baseline_m:.4f} m → {candidate:.4f} m'
        )
        self.current_baseline_m = candidate
        self._recompute_hole_flags()

    def _recompute_hole_flags(self):
        threshold = self.current_baseline_m + self.hole_tolerance_m
        self.points_by_cell = {
            k: (x, y, z, z > threshold)
            for k, (x, y, z, _) in self.points_by_cell.items()
        }

    # ──────────────────────────────────────────────────────────────────────────
    def update_scan_state(self, dt: float):
        if self.state == self.SCANNING_X:
            self.current_x_mm += self.x_speed_mm_s * dt * self.direction_x
            at_right = self.direction_x ==  1 and self.current_x_mm >= self.x_length_mm
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
                    self.get_logger().info('ToF scan complete.')
                    self._publish_result_once()
                else:
                    self.last_grid_cell = None
                    self.state = self.SCANNING_X

    def update_costmap(self, distance_m: float, x_m: float, y_m: float):
        grid_x = int(round(x_m / self.cube_size_m))
        grid_y = int(round(y_m / self.cube_size_m))
        cell = (grid_x, grid_y)
        if cell == self.last_grid_cell:
            return

        z_m = max(0.0, distance_m)
        is_hole = z_m > self.current_baseline_m + self.hole_tolerance_m
        self.points_by_cell[cell] = (x_m, y_m, z_m, is_hole)
        self.last_grid_cell = cell

    # ──────────────────────────────────────────────────────────────────────────
    def _find_hole_clusters(self) -> list:
        hole_keys = {k for k, (_, _, _, h) in self.points_by_cell.items() if h}
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
                queue.extend([(gx+1,gy),(gx-1,gy),(gx,gy+1),(gx,gy-1)])
            clusters.append([
                (self.points_by_cell[k][0],
                 self.points_by_cell[k][1],
                 self.points_by_cell[k][2]) for k in keys
            ])
        return clusters

    def _cluster_stats(self, cluster: list) -> tuple:
        """Returns (vol_m3, cx, cy, cz).  cz is negative (below floor)."""
        area = self.cube_size_m ** 2
        tv = cx = cy = cz = 0.0
        for (x, y, d) in cluster:
            dep  = d - self.current_baseline_m
            vol  = area * dep
            tv  += vol
            cx  += x * vol
            cy  += y * vol
            cz  += -(self.current_baseline_m + dep / 2.0) * vol
        if tv == 0.0:
            return 0.0, 0.0, 0.0, 0.0
        return tv, cx/tv, cy/tv, cz/tv

    # ──────────────────────────────────────────────────────────────────────────
    def _publish_result_once(self):
        """
        Called exactly once when state → DONE.
        Publishes a structured JSON String on tof_costmap/result.
        The visualiser node subscribes to this and builds its display.
        """
        clusters = self._find_hole_clusters()

        payload = {
            'baseline_m':   self.current_baseline_m,
            'scan_step_m':  self.cube_size_m,
            'total_holes':  len(clusters),
            'clusters': []
        }

        for i, cluster in enumerate(clusters):
            vol_m3, cx, cy, cz = self._cluster_stats(cluster)
            payload['clusters'].append({
                'id':         i + 1,
                'cells':      len(cluster),
                'volume_cm3': round(vol_m3 * 1e6, 4),
                'centroid':   {'x': round(cx,4), 'y': round(cy,4), 'z': round(cz,4)},
                'points': [
                    {'x': round(x,4),
                     'y': round(y,4),
                     'depth_m': round(d - self.current_baseline_m, 5)}
                    for (x, y, d) in cluster
                ]
            })

        msg      = String()
        msg.data = json.dumps(payload)
        self.result_pub.publish(msg)
        self._result_published = True

        # Human-readable log summary
        lines = [
            f'\n━━━ Scan Result ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━',
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
        m.pose.position.z    = 0.0   # Z=0 is the floor reference
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
        """
        Blue-family gradient mapped over ±ground_variance_range_m.
          raised (dist < baseline) : icy white-blue
          nominal (dist ≈ baseline): blue-grey
          slight dip (near thresh) : amber warning
        """
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
        """
        Full-spectrum gradient across hole_depth_range_m:
          shallow : bright yellow
          mid     : vivid orange-red
          deep    : dark crimson
        """
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
        """
        Z-axis layout — sensor at origin, Z=0 is the floor reference plane.

        GROUND cubes (dist <= baseline + tolerance)
        ────────────────────────────────────────────
          Each ground cube reaches DOWN from the sensor to the actual measured
          surface.  The cube's BOTTOM face sits at Z=0 (floor reference) and
          extends UPWARD (toward the sensor) by cube_size.
          This makes every ground cube a "column" that connects the floor plane
          to the sensor — identical in concept to hole cubes, just going up.

          display_z_centre = cube_size / 2     (always the same fixed Z)
          scale_z          = cube_size          (fixed)

          Surface variation is shown entirely through colour — see _ground_color.

        HOLE cubes (dist > baseline + tolerance)
        ─────────────────────────────────────────
          The void hangs BELOW Z=0 by the excess depth:
            depth_m          = dist_m - baseline_m
            display_z_centre = -(depth_m / 2)
            scale_z          = depth_m   (min = cube_size)

          Top face is flush with Z=0, so ground and hole cubes share the same
          floor plane seam — the costmap is one continuous connected surface.
        """
        clusters     = self._find_hole_clusters()
        marker_array = MarkerArray()

        del_m = Marker()
        del_m.header.stamp    = stamp
        del_m.header.frame_id = self.frame_id
        del_m.ns              = 'tof_costmap'
        del_m.id              = 0
        del_m.action          = Marker.DELETEALL
        marker_array.markers.append(del_m)

        half    = self.cube_size_m / 2.0
        next_id = 1

        for (_gx, _gy), (x_m, y_m, z_m, is_hole) in self.points_by_cell.items():
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
                # Void below Z=0: top at 0, bottom at -depth, centre at -depth/2
                depth_m           = z_m - self.current_baseline_m
                height            = max(depth_m, self.cube_size_m)
                m.scale.z         = height
                m.pose.position.z = -(height / 2.0)
                m.color           = self._hole_color(depth_m)
            else:
                # Ground column: bottom at Z=0, top at +cube_size, centre at +half
                # All ground cubes share the same Z regardless of measured distance.
                # Colour encodes the surface variation instead.
                m.scale.z         = self.cube_size_m
                m.pose.position.z = half
                m.color           = self._ground_color(z_m)

            marker_array.markers.append(m)

        # ── TEXT labels at hole cluster centroids ─────────────────────────────
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
            t.pose.position.z = cz + 0.03        # 30 mm above centroid
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