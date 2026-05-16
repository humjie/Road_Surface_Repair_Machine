"""
one_shot_run.py  —  ToF scan + fill orchestration

What this version does
======================
1. Waits for the stepper firmware to finish homing.
2. Sends the real stepper 500 steps back (-X) and 500 steps left (-Y) as a
   pre-move before the scan starts.
3. Raster-scans the area by publishing /target_xy waypoints (real stepper moves).
4. At each scan cell the live /tof_data reading is logged, but the costmap depth
   is synthesised from Gaussian blobs (SIM_HOLES) so the visualiser always shows
   clean, well-defined holes regardless of sensor noise.
5. Clusters the synthetic depth map, publishes:
     /tof_costmap              — JSON result
     /tof_result/hole_markers  — one CUBE per cell, sunk below z=0 ground plane
     /tof_result/text_markers  — volume + centroid text per cluster
     /tof_result/ground_plane  — semi-transparent ground reference quad
6. Moves the real stepper to each hole centroid and fires the pump.
"""

from __future__ import annotations

import json
import math
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Point, PointStamped
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import ColorRGBA, Int32, String
from visualization_msgs.msg import Marker, MarkerArray


# ── Synthetic hole definitions ────────────────────────────────────────────────
# Each entry: (cx_steps, cy_steps, radius_steps, max_depth_m)
SIM_HOLES: List[Tuple[float, float, float, float]] = [
    (  0, 0, 0, 0, 0, 0, 0 ),
    (0, 0, 2000.0,  3000.0, 3500.0, 25,0),   # medium hole, top-right area
    ( 0,0,-4000.0, -2000.0, 2000.0, 18,0),   # small hole,  bottom-left
    (  0,8000.0, -6000.0, 500.0, 40,0,0),   # large hole,  bottom-right
]

# Pre-move offset applied once after homing (step units).
PRE_MOVE_X_OFFSET = -500.0
PRE_MOVE_Y_OFFSET = -500.0


# ── Colour helpers (matches hole_visualiser_node.py palette) ─────────────────

def _lerp3(t: float, c0: tuple, c1: tuple) -> tuple:
    return tuple(c0[i] + (c1[i] - c0[i]) * t for i in range(3))


def _hole_color(depth_m: float, depth_range_m: float = 0.10) -> ColorRGBA:
    """Amber → orange → dark-red heat-map, matching hole_visualiser_node."""
    dr = max(depth_range_m, 1e-6)
    t  = min(1.0, depth_m / dr)
    if t < 0.5:
        r, g, b = _lerp3(t * 2.0, (1.00, 0.85, 0.05), (1.00, 0.28, 0.05))
    else:
        r, g, b = _lerp3((t - 0.5) * 2.0, (1.00, 0.28, 0.05), (0.50, 0.02, 0.02))
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=0.78 + 0.20 * t)


def _cluster_color(idx: int) -> ColorRGBA:
    palette = [
        (0.9, 0.6, 0.1), (0.2, 0.8, 0.6), (0.7, 0.3, 0.9),
        (0.2, 0.6, 1.0), (1.0, 0.4, 0.4), (0.5, 0.9, 0.3),
        (0.9, 0.2, 0.7), (0.3, 0.4, 0.9),
    ]
    r, g, b = palette[idx % len(palette)]
    return ColorRGBA(r=r, g=g, b=b, a=1.0)


# ─────────────────────────────────────────────────────────────────────────────

class OneShotRun(Node):
    def __init__(self):
        super().__init__('tof_one_shot_run')

        # ── Topics ────────────────────────────────────────────────────────────
        self.declare_parameter('input_topic',       '/tof_data')
        self.declare_parameter('current_xy_topic',  '/current_xy_pos')
        self.declare_parameter('target_xy_topic',   '/target_xy')
        self.declare_parameter('pump_cmd_topic',    '/pump_cmd')
        self.declare_parameter('result_topic',      '/tof_costmap')
        self.declare_parameter('frame_id',          'tof_sensor_link')

        # ── Geometry / sensing ────────────────────────────────────────────────
        self.declare_parameter('sensor_to_ground_m',  0.03)
        self.declare_parameter('hole_tolerance_m',    0.005)
        self.declare_parameter('hole_depth_range_m',  0.10)   # for colour scaling

        # ── Scan window (step units, homed frame) ─────────────────────────────
        self.declare_parameter('scan_x_start_steps', -1600.0)
        self.declare_parameter('scan_x_end_steps',    1600.0)
        self.declare_parameter('scan_y_start_steps', -1600.0)
        self.declare_parameter('scan_y_end_steps',    1600.0)
        self.declare_parameter('scan_step_units',     400.0)
        self.declare_parameter('steps_per_mm',        40.0)

        # ── Timing ────────────────────────────────────────────────────────────
        self.declare_parameter('sample_window_s',  0.25)
        self.declare_parameter('move_timeout_s',   20.0)
        self.declare_parameter('home_timeout_s',   45.0)
        self.declare_parameter('fill_settle_s',    0.35)

        # ── Pump ──────────────────────────────────────────────────────────────
        self.declare_parameter('pump_rate_cm3_s', 1.0)
        self.declare_parameter('min_pump_ms',     200)
        self.declare_parameter('max_pump_ms',     60000)

        # ── Resolve ───────────────────────────────────────────────────────────
        self.input_topic       = self.get_parameter('input_topic').value
        self.current_xy_topic  = self.get_parameter('current_xy_topic').value
        self.target_xy_topic   = self.get_parameter('target_xy_topic').value
        self.pump_cmd_topic    = self.get_parameter('pump_cmd_topic').value
        self.result_topic      = self.get_parameter('result_topic').value
        self.frame_id          = self.get_parameter('frame_id').value

        self.baseline_m        = float(self.get_parameter('sensor_to_ground_m').value)
        self.hole_tolerance_m  = float(self.get_parameter('hole_tolerance_m').value)
        self.hole_depth_range_m= float(self.get_parameter('hole_depth_range_m').value)

        self.scan_x_start      = float(self.get_parameter('scan_x_start_steps').value)
        self.scan_x_end        = float(self.get_parameter('scan_x_end_steps').value)
        self.scan_y_start      = float(self.get_parameter('scan_y_start_steps').value)
        self.scan_y_end        = float(self.get_parameter('scan_y_end_steps').value)
        self.scan_step_units   = float(self.get_parameter('scan_step_units').value)
        self.steps_per_mm      = float(self.get_parameter('steps_per_mm').value)

        self.sample_window_s   = float(self.get_parameter('sample_window_s').value)
        self.move_timeout_s    = float(self.get_parameter('move_timeout_s').value)
        self.home_timeout_s    = float(self.get_parameter('home_timeout_s').value)
        self.fill_settle_s     = float(self.get_parameter('fill_settle_s').value)

        self.pump_rate_cm3_s   = float(self.get_parameter('pump_rate_cm3_s').value)
        self.min_pump_ms       = int(self.get_parameter('min_pump_ms').value)
        self.max_pump_ms       = int(self.get_parameter('max_pump_ms').value)

        # Cell side length in metres (used for cube scale + area calculations)
        self.cell_side_m = (self.scan_step_units / max(self.steps_per_mm, 1e-9)) / 1000.0

        # ── Publishers ────────────────────────────────────────────────────────
        self.target_pub      = self.create_publisher(Point,       self.target_xy_topic,            10)
        self.pump_pub        = self.create_publisher(Int32,       self.pump_cmd_topic,              10)
        self.result_pub      = self.create_publisher(String,      self.result_topic,                10)
        self.hole_pub        = self.create_publisher(MarkerArray, '/tof_result/hole_markers',       10)
        self.text_pub        = self.create_publisher(MarkerArray, '/tof_result/text_markers',       10)
        self.ground_pub      = self.create_publisher(MarkerArray, '/tof_result/ground_plane',       10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.range_sub = self.create_subscription(
            Range, self.input_topic, self._range_callback, 10)
        self.current_xy_sub = self.create_subscription(
            PointStamped, self.current_xy_topic, self._current_xy_callback, 10)
        self.move_done_sub = self.create_subscription(
            String, '/move_done', self._move_done_callback, 10)

        self.timer        = self.create_timer(0.05, self._tick)
        # Republish markers at 1 Hz once the scan result is available so
        # Foxglove subscribers that connect late still see the costmap.
        self.marker_timer = self.create_timer(1.0,  self._republish_markers)

        # ── State machine ─────────────────────────────────────────────────────
        self._phase                = 'wait_home'
        self._home_seen            = False
        self._deadline             = self._now_sec() + self.home_timeout_s

        self._scan_path            = self._build_scan_path()
        self._scan_index           = 0
        self._current_target_steps : Optional[Tuple[float, float]] = None
        self._move_deadline        : Optional[float] = None
        self._collect_until        : Optional[float] = None
        self._settle_until         : Optional[float] = None

        # Synthetic depth accumulator keyed by grid cell (gx, gy)
        self._cell_accum           : Dict[Tuple[int, int], Dict] = {}
        self._current_xy           : Optional[Tuple[float, float]] = None

        self._clusters             : List[Dict] = []
        self._fill_queue           : List[Dict] = []
        self._fill_index           = 0
        self._pump_until           : Optional[float] = None
        self._result_ready         : bool = False   # set True after first publish

        self.get_logger().info(
            'OneShotRun started\n'
            f'  Input topic     : {self.input_topic}\n'
            f'  Target XY topic : {self.target_xy_topic}\n'
            f'  Pump topic      : {self.pump_cmd_topic}\n'
            f'  Result topic    : {self.result_topic}\n'
            f'  Hole markers    : /tof_result/hole_markers\n'
            f'  Text markers    : /tof_result/text_markers\n'
            f'  Ground plane    : /tof_result/ground_plane\n'
            f'  Scan X range    : [{self.scan_x_start}, {self.scan_x_end}] steps\n'
            f'  Scan Y range    : [{self.scan_y_start}, {self.scan_y_end}] steps\n'
            f'  Scan grid       : {self.scan_step_units} steps '
            f'= {self.cell_side_m * 1000:.1f} mm/cell\n'
            f'  Steps per mm    : {self.steps_per_mm}\n'
            f'  Baseline        : {self.baseline_m:.4f} m\n'
            f'  Hole tolerance  : {self.hole_tolerance_m * 1000:.1f} mm\n'
            f'  Pre-move offset : ({PRE_MOVE_X_OFFSET:+.0f}, {PRE_MOVE_Y_OFFSET:+.0f}) steps\n'
            f'  Sim holes       : {len(SIM_HOLES)}\n'
            f'  Move timeout    : {self.move_timeout_s} s\n'
            f'  Home timeout    : {self.home_timeout_s} s'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _current_xy_callback(self, msg: PointStamped):
        self._current_xy = (float(msg.point.x), float(msg.point.y))

    def _move_done_callback(self, msg: String):
        data = msg.data.strip()

        if self._phase == 'wait_home':
            if data == 'home':
                self._home_seen = True
                self.get_logger().info(
                    f'Homing complete. Pre-moving '
                    f'({PRE_MOVE_X_OFFSET:+.0f}, {PRE_MOVE_Y_OFFSET:+.0f}) steps.')
                self._send_pre_move()
            elif data == 'home_fail':
                self.get_logger().error('Firmware reported home_fail. Aborting.')
                self._phase = 'done'
            return

        if data != 'ok':
            return

        if self._phase == 'pre_move':
            self.get_logger().info('Pre-move complete. Starting scan.')
            self._phase = 'scan_move'
            self._send_next_scan_target()
            return

        if self._phase == 'scan_move':
            self._phase = 'scan_collect'
            self._collect_until = self._now_sec() + self.sample_window_s
            self._inject_synthetic_depth()
            return

        if self._phase == 'fill_move':
            self._phase = 'fill_settle'
            self._settle_until = self._now_sec() + self.fill_settle_s
            return

    def _range_callback(self, msg: Range):
        """Log live ToF readings. Does NOT feed the costmap (synthetic used)."""
        self.get_logger().info(
            f'[ToF] {float(msg.range):.4f} m  (phase: {self._phase})')

    # ── Synthetic depth injection ──────────────────────────────────────────────

    def _inject_synthetic_depth(self):
        if self._current_target_steps is None:
            return
        sx, sy  = self._current_target_steps
        depth_m = self._synthetic_depth(sx, sy)
        gx      = int(round(sx / self.scan_step_units))
        gy      = int(round(sy / self.scan_step_units))
        cell    = (gx, gy)
        entry   = self._cell_accum.setdefault(
            cell,
            {'x_steps': sx, 'y_steps': sy, 'sum_depth': 0.0, 'count': 0.0},
        )
        entry['sum_depth'] += depth_m
        entry['count']     += 1.0

    def _synthetic_depth(self, x_steps: float, y_steps: float) -> float:
        depth = 0.0
        for cx, cy, r, max_d in SIM_HOLES:
            d2    = (x_steps - cx) ** 2 + (y_steps - cy) ** 2
            sigma = r / 2.0
            depth += max_d * math.exp(-d2 / (2.0 * sigma ** 2))
        return depth

    # ── Scan path ─────────────────────────────────────────────────────────────

    def _build_scan_path(self) -> List[Tuple[float, float]]:
        step        = max(self.scan_step_units, 1.0)
        x_positions = list(self._frange(self.scan_x_start, self.scan_x_end, step))
        y_positions = list(self._frange(self.scan_y_start, self.scan_y_end, step))
        y_positions.reverse()
        path: List[Tuple[float, float]] = []
        forward = False
        for y in y_positions:
            row = list(reversed(x_positions)) if not forward else x_positions
            for x in row:
                path.append((x, y))
            forward = not forward
        return path

    @staticmethod
    def _frange(start: float, stop: float, step: float):
        v = start
        while v <= stop + 1e-6:
            yield round(v, 6)
            v += step

    def _send_target_steps(self, x_steps: float, y_steps: float):
        msg   = Point()
        msg.x = float(x_steps)
        msg.y = float(y_steps)
        msg.z = 0.0
        self.target_pub.publish(msg)
        self._current_target_steps = (x_steps, y_steps)
        self._move_deadline        = self._now_sec() + self.move_timeout_s

    def _send_pre_move(self):
        if self._current_xy is not None:
            base_x, base_y = self._current_xy
        else:
            base_x, base_y = 0.0, 0.0
            self.get_logger().warn('No /current_xy_pos yet; pre-moving from (0, 0).')
        tx = base_x + PRE_MOVE_X_OFFSET
        ty = base_y + PRE_MOVE_Y_OFFSET
        self.get_logger().info(f'Pre-move target: ({tx:.0f}, {ty:.0f}) steps')
        self._phase = 'pre_move'
        self._send_target_steps(tx, ty)

    def _send_next_scan_target(self):
        if self._scan_index >= len(self._scan_path):
            self._finalise_scan()
            return
        x_steps, y_steps = self._scan_path[self._scan_index]
        self._scan_index += 1
        self._send_target_steps(x_steps, y_steps)

    # ── Fill ──────────────────────────────────────────────────────────────────

    def _finalise_scan(self):
        self._clusters = self._find_clusters()
        self._publish_result(self._build_result_payload())
        self._publish_all_markers(self._clusters)
        self._result_ready = True

        self._fill_queue = sorted(
            self._clusters,
            key=lambda item: float(item['volume_cm3']),
            reverse=True,
        )
        self._fill_index = 0

        if not self._fill_queue:
            self.get_logger().info('Scan finished, no holes found. Shutting down.')
            self._phase = 'done'
            return

        self.get_logger().info(
            f'Scan finished. Filling {len(self._fill_queue)} hole cluster(s).')
        self._phase = 'fill_move'
        self._send_next_fill_target()

    def _send_next_fill_target(self):
        if self._fill_index >= len(self._fill_queue):
            self._phase = 'done'
            self.get_logger().info('All clusters filled. Shutting down.')
            return
        cluster  = self._fill_queue[self._fill_index]
        centroid = cluster['centroid']
        self.get_logger().info(
            f'Moving to hole {cluster["id"]} centroid: '
            f'({centroid["x_steps"]:.0f}, {centroid["y_steps"]:.0f}) steps')
        self._send_target_steps(
            float(centroid['x_steps']), float(centroid['y_steps']))

    def _publish_pump(self, volume_cm3: float):
        if self.pump_rate_cm3_s <= 0.0:
            duration_ms = self.min_pump_ms
        else:
            duration_ms = int(round((volume_cm3 / self.pump_rate_cm3_s) * 1000.0))
            duration_ms = max(self.min_pump_ms, min(self.max_pump_ms, duration_ms))
        msg      = Int32()
        msg.data = duration_ms
        self.pump_pub.publish(msg)
        self._pump_until = self._now_sec() + max(0.2, duration_ms / 1000.0)
        self.get_logger().info(f'Pumping {duration_ms} ms for {volume_cm3:.2f} cm³')

    # ── Clustering ────────────────────────────────────────────────────────────

    def _find_clusters(self) -> List[Dict]:
        cell_area_m2 = self.cell_side_m ** 2

        hole_cells = {
            cell for cell, entry in self._cell_accum.items()
            if (entry['sum_depth'] / max(entry['count'], 1.0)) > self.hole_tolerance_m
        }
        visited  = set()
        clusters : List[Dict] = []

        def neighbors(cell: Tuple[int, int]):
            x, y = cell
            yield (x + 1, y); yield (x - 1, y)
            yield (x, y + 1); yield (x, y - 1)

        for start in sorted(hole_cells):
            if start in visited:
                continue
            stack     = [start]
            component : List[Tuple[int, int]] = []
            visited.add(start)
            while stack:
                cell = stack.pop()
                component.append(cell)
                for nxt in neighbors(cell):
                    if nxt in hole_cells and nxt not in visited:
                        visited.add(nxt)
                        stack.append(nxt)

            points    : list = []
            volume_m3 = 0.0
            cx_s = cy_s = cx_m = cy_m = cz_m = 0.0

            for cell in component:
                entry   = self._cell_accum[cell]
                depth_m = entry['sum_depth'] / max(entry['count'], 1.0)
                x_steps = float(entry['x_steps'])
                y_steps = float(entry['y_steps'])
                x_m     = (x_steps / max(self.steps_per_mm, 1e-9)) / 1000.0
                y_m     = (y_steps / max(self.steps_per_mm, 1e-9)) / 1000.0

                points.append({
                    'x_m':     round(x_m,     4),
                    'y_m':     round(y_m,     4),
                    'x_steps': round(x_steps, 2),
                    'y_steps': round(y_steps, 2),
                    'depth_m': round(depth_m, 4),
                })
                vol        = depth_m * cell_area_m2
                volume_m3 += vol
                cx_s      += x_steps * vol;  cy_s += y_steps * vol
                cx_m      += x_m     * vol;  cy_m += y_m     * vol
                cz_m      += (-(depth_m / 2.0)) * vol  # mid-depth below floor=0

            if not points or volume_m3 <= 0.0:
                continue

            clusters.append({
                'id':         len(clusters) + 1,
                'cells':      len(points),
                'volume_cm3': round(volume_m3 * 1e6, 4),
                'centroid': {
                    'x_m':     round(cx_m / volume_m3, 4),
                    'y_m':     round(cy_m / volume_m3, 4),
                    'z_m':     round(cz_m / volume_m3, 4),
                    'x_steps': round(cx_s / volume_m3, 2),
                    'y_steps': round(cy_s / volume_m3, 2),
                },
                'points': points,
            })

        return clusters

    def _build_result_payload(self) -> Dict:
        return {
            'frame_id':        self.frame_id,
            'baseline_m':      round(self.baseline_m, 4),
            'scan_step_m':     round(self.cell_side_m, 6),
            'scan_step_units': self.scan_step_units,
            'steps_per_mm':    self.steps_per_mm,
            'total_holes':     len(self._clusters),
            'clusters':        self._clusters,
        }

    def _publish_result(self, payload: Dict):
        msg      = String()
        msg.data = json.dumps(payload)
        self.result_pub.publish(msg)
        self.get_logger().info(
            f'Published /tof_costmap ({len(self._clusters)} cluster(s)).')

    # ── Markers ───────────────────────────────────────────────────────────────

    def _publish_all_markers(self, clusters: List[Dict]):
        stamp = self.get_clock().now().to_msg()
        self._publish_hole_markers(clusters, stamp)
        self._publish_text_markers(clusters, stamp)
        self._publish_ground_plane(stamp)

    # ── 1. Hole cubes + bounding-box outlines ─────────────────────────────────

    def _publish_hole_markers(self, clusters: List[Dict], stamp):
        """
        One CUBE per scan cell, sunk below z=0 (the ground plane).
        The cube sits at z = -(depth/2) and has height = depth,
        so its top face is flush with z=0 and its bottom is at z=-depth.
        Also draws a LINE_LIST bounding box per cluster (no z-fighting).
        """
        ma = MarkerArray()

        # DELETEALL first (same message → no flicker)
        del_m                 = Marker()
        del_m.header.stamp    = stamp
        del_m.header.frame_id = self.frame_id
        del_m.ns              = 'hole_cubes'
        del_m.id              = 0
        del_m.action          = Marker.DELETEALL
        ma.markers.append(del_m)

        if not clusters:
            self.hole_pub.publish(ma)
            return

        next_id = 1
        s       = self.cell_side_m

        for ci, cluster in enumerate(clusters):
            points = cluster['points']

            # ── Per-cell depth cube ───────────────────────────────────────────
            for pt in points:
                x_m     = float(pt['x_m'])
                y_m     = float(pt['y_m'])
                depth_m = float(pt['depth_m'])
                height  = max(depth_m, s * 0.1)   # minimum visible sliver

                m                    = Marker()
                m.header.stamp       = stamp
                m.header.frame_id    = self.frame_id
                m.ns                 = 'hole_cubes'
                m.id                 = next_id;  next_id += 1
                m.type               = Marker.CUBE
                m.action             = Marker.ADD
                m.pose.orientation.w = 1.0
                m.lifetime.sec       = 0
                # Position: centred at mid-depth below the floor
                m.pose.position.x    = x_m
                m.pose.position.y    = y_m
                m.pose.position.z    = -(height / 2.0)
                # Scale: cell footprint × actual depth
                m.scale.x            = s
                m.scale.y            = s
                m.scale.z            = height
                m.color              = _hole_color(depth_m, self.hole_depth_range_m)
                ma.markers.append(m)

            # ── Bounding-box LINE_LIST outline ────────────────────────────────
            if points:
                xs      = [float(p['x_m'])     for p in points]
                ys      = [float(p['y_m'])     for p in points]
                ds      = [float(p['depth_m']) for p in points]
                x_min   = min(xs) - s / 2.0
                x_max   = max(xs) + s / 2.0
                y_min   = min(ys) - s / 2.0
                y_max   = max(ys) + s / 2.0
                z_bot   = -max(ds) * 1.05          # slightly below deepest cell
                z_top   = s * 0.1                  # slightly above ground

                corners = [
                    (x_min, y_min, z_bot), (x_max, y_min, z_bot),
                    (x_max, y_max, z_bot), (x_min, y_max, z_bot),
                    (x_min, y_min, z_top), (x_max, y_min, z_top),
                    (x_max, y_max, z_top), (x_min, y_max, z_top),
                ]
                edges = [
                    (0, 1), (1, 2), (2, 3), (3, 0),  # bottom ring
                    (4, 5), (5, 6), (6, 7), (7, 4),  # top ring
                    (0, 4), (1, 5), (2, 6), (3, 7),  # vertical edges
                ]
                line                    = Marker()
                line.header.stamp       = stamp
                line.header.frame_id    = self.frame_id
                line.ns                 = 'hole_cubes'
                line.id                 = next_id;  next_id += 1
                line.type               = Marker.LINE_LIST
                line.action             = Marker.ADD
                line.pose.orientation.w = 1.0
                line.lifetime.sec       = 0
                line.scale.x            = 0.002       # 2 mm line thickness
                cc                      = _cluster_color(ci)
                line.color              = cc
                line.color.a            = 0.9
                for a, b in edges:
                    pa = Point(); pa.x, pa.y, pa.z = corners[a]
                    pb = Point(); pb.x, pb.y, pb.z = corners[b]
                    line.points.append(pa)
                    line.points.append(pb)
                ma.markers.append(line)

        self.hole_pub.publish(ma)

    # ── 2. Text labels with volume + centroid ─────────────────────────────────

    def _publish_text_markers(self, clusters: List[Dict], stamp):
        """
        One TEXT_VIEW_FACING label per cluster, anchored just above the
        ground plane.  Shows: cluster ID, volume (cm³), centroid (x, y, z).
        """
        ma = MarkerArray()

        del_m                 = Marker()
        del_m.header.stamp    = stamp
        del_m.header.frame_id = self.frame_id
        del_m.ns              = 'hole_text'
        del_m.id              = 0
        del_m.action          = Marker.DELETEALL
        ma.markers.append(del_m)

        if not clusters:
            self.text_pub.publish(ma)
            return

        next_id    = 1
        text_scale = max(self.cell_side_m * 2.5, 0.012)  # readable in Foxglove

        for ci, cluster in enumerate(clusters):
            cid     = cluster['id']
            cells   = cluster['cells']
            vol_cm3 = float(cluster['volume_cm3'])
            cx_m    = float(cluster['centroid']['x_m'])
            cy_m    = float(cluster['centroid']['y_m'])
            cz_m    = float(cluster['centroid']['z_m'])
            bc      = _cluster_color(ci)

            # ── Main info label ───────────────────────────────────────────────
            t                    = Marker()
            t.header.stamp       = stamp
            t.header.frame_id    = self.frame_id
            t.ns                 = 'hole_text'
            t.id                 = next_id;  next_id += 1
            t.type               = Marker.TEXT_VIEW_FACING
            t.action             = Marker.ADD
            t.pose.orientation.w = 1.0
            t.lifetime.sec       = 0
            t.pose.position.x    = cx_m
            t.pose.position.y    = cy_m
            # Anchor above ground so text is always visible regardless of depth
            t.pose.position.z    = self.cell_side_m + text_scale * 2.0
            t.scale.z            = text_scale
            t.color              = ColorRGBA(r=bc.r, g=bc.g, b=bc.b, a=1.0)
            t.text = (
                f'Cluster #{cid}\n'
                f'Vol : {vol_cm3:.2f} cm³\n'
                f'Cells: {cells}\n'
                f'Centroid:\n'
                f'  x={cx_m:.4f} m\n'
                f'  y={cy_m:.4f} m\n'
                f'  z={cz_m:.4f} m'
            )
            ma.markers.append(t)

            # ── Centroid sphere ───────────────────────────────────────────────
            sp                    = Marker()
            sp.header.stamp       = stamp
            sp.header.frame_id    = self.frame_id
            sp.ns                 = 'hole_text'
            sp.id                 = next_id;  next_id += 1
            sp.type               = Marker.SPHERE
            sp.action             = Marker.ADD
            sp.pose.position.x    = cx_m
            sp.pose.position.y    = cy_m
            sp.pose.position.z    = cz_m
            sp.pose.orientation.w = 1.0
            sp.scale.x = sp.scale.y = sp.scale.z = self.cell_side_m * 0.6
            sp.color              = ColorRGBA(r=bc.r, g=bc.g, b=bc.b, a=1.0)
            sp.lifetime.sec       = 0
            ma.markers.append(sp)

            # ── Arrow: ground → centroid ──────────────────────────────────────
            arr                    = Marker()
            arr.header.stamp       = stamp
            arr.header.frame_id    = self.frame_id
            arr.ns                 = 'hole_text'
            arr.id                 = next_id;  next_id += 1
            arr.type               = Marker.ARROW
            arr.action             = Marker.ADD
            arr.pose.orientation.w = 1.0
            arr.lifetime.sec       = 0
            # Arrow from z=0 straight down to the centroid mid-depth
            arr.points = [
                Point(x=cx_m, y=cy_m, z=0.0),
                Point(x=cx_m, y=cy_m, z=cz_m),
            ]
            arr.scale.x = self.cell_side_m * 0.15   # shaft diameter
            arr.scale.y = self.cell_side_m * 0.30   # head diameter
            arr.scale.z = self.cell_side_m * 0.30   # head length
            arr.color   = ColorRGBA(r=bc.r, g=bc.g, b=bc.b, a=0.9)
            ma.markers.append(arr)

        self.text_pub.publish(ma)

    # ── 3. Ground plane reference ─────────────────────────────────────────────

    def _publish_ground_plane(self, stamp):
        """
        Semi-transparent flat quad at z=0 covering the full scan area.
        Gives spatial context so the cubes below it look like real holes.
        """
        ma = MarkerArray()

        del_m                 = Marker()
        del_m.header.stamp    = stamp
        del_m.header.frame_id = self.frame_id
        del_m.ns              = 'ground_plane'
        del_m.id              = 0
        del_m.action          = Marker.DELETEALL
        ma.markers.append(del_m)

        # Width / height of the scan area in metres
        x_span_m = ((self.scan_x_end - self.scan_x_start)
                    / max(self.steps_per_mm, 1e-9)) / 1000.0
        y_span_m = ((self.scan_y_end - self.scan_y_start)
                    / max(self.steps_per_mm, 1e-9)) / 1000.0
        cx_m     = ((self.scan_x_start + self.scan_x_end) / 2.0
                    / max(self.steps_per_mm, 1e-9)) / 1000.0
        cy_m     = ((self.scan_y_start + self.scan_y_end) / 2.0
                    / max(self.steps_per_mm, 1e-9)) / 1000.0

        g                    = Marker()
        g.header.stamp       = stamp
        g.header.frame_id    = self.frame_id
        g.ns                 = 'ground_plane'
        g.id                 = 1
        g.type               = Marker.CUBE
        g.action             = Marker.ADD
        g.pose.orientation.w = 1.0
        g.lifetime.sec       = 0
        g.pose.position.x    = cx_m
        g.pose.position.y    = cy_m
        g.pose.position.z    = 0.0
        g.scale.x            = x_span_m
        g.scale.y            = y_span_m
        g.scale.z            = 0.001              # 1 mm thin slab
        g.color              = ColorRGBA(r=0.85, g=0.90, b=0.85, a=0.18)
        ma.markers.append(g)

        self.ground_pub.publish(ma)

    # ── Continuous marker republish ───────────────────────────────────────────

    def _republish_markers(self):
        """Called at 1 Hz. Re-sends all marker topics after the scan completes
        so Foxglove subscribers that connect late receive the costmap."""
        if not self._result_ready:
            return
        self._publish_all_markers(self._clusters)

    # ── Tick (state machine) ──────────────────────────────────────────────────

    def _tick(self):
        now = self._now_sec()

        if self._phase == 'wait_home':
            if now >= self._deadline and not self._home_seen:
                self.get_logger().warn('Home timeout — sending pre-move anyway.')
                self._send_pre_move()
            return

        if (self._phase == 'pre_move'
                and self._move_deadline is not None
                and now > self._move_deadline):
            self.get_logger().error('Timed out on pre-move. Aborting.')
            self._phase = 'done'
            return

        if (self._phase == 'scan_move'
                and self._move_deadline is not None
                and now > self._move_deadline):
            self.get_logger().error('Timed out waiting for scan move.')
            self._phase = 'done'
            return

        if (self._phase == 'scan_collect'
                and self._collect_until is not None
                and now >= self._collect_until):
            self._collect_until = None
            self._phase         = 'scan_move'
            self._send_next_scan_target()
            return

        if (self._phase == 'fill_move'
                and self._move_deadline is not None
                and now > self._move_deadline):
            self.get_logger().error('Timed out waiting for fill move.')
            self._phase = 'done'
            return

        if (self._phase == 'fill_settle'
                and self._settle_until is not None
                and now >= self._settle_until):
            self._settle_until = None
            self._phase        = 'fill_pump'
            volume = float(self._fill_queue[self._fill_index]['volume_cm3'])
            self._publish_pump(volume)
            return

        if (self._phase == 'fill_pump'
                and self._pump_until is not None
                and now >= self._pump_until):
            stop      = Int32()
            stop.data = 0
            self.pump_pub.publish(stop)
            self._pump_until  = None
            self._fill_index += 1
            if self._fill_index >= len(self._fill_queue):
                self._phase = 'done'
                self.get_logger().info('Filling complete.')
            else:
                self._phase = 'fill_move'
                self._send_next_fill_target()
            return

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = OneShotRun()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('OneShotRun interrupted, shutting down.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()