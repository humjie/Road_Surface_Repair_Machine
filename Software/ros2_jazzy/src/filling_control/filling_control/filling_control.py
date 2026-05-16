"""
filling_control.py
━━━━━━━━━━━━━━━━━━
Operator-gated filling orchestrator.

System flow
───────────
  1. Operator triggers homing at startup:
       /change_main_state = 'homing'  → 'homing' → stepper homes → 'free'
  2. Operator triggers scan:
       /change_main_state = 'scanning' → 'scanning' → tof_costmap runs
  3. Scan ends:
       tof_costmap publishes /change_main_state = 'wait_for_fill'
       tof_visualiser publishes /tof_result with refined hole data
       filling_control plans holes but DOES NOT FILL
       (parked, waiting for operator confirmation in Foxglove)
  4. Operator presses 'Start Filling' button in Foxglove:
       /change_main_state = 'filling' → 'filling'
       filling_control: move-to-home → for each hole: move → lower → pump → raise
  5. On completion:
       filling_control publishes /change_main_state = 'free'

Key behavioural changes vs. previous version
─────────────────────────────────────────────
  * No longer auto-requests 'homing' or 'filling' state transitions.
    Those are operator-driven now.
  * Only acts when /main_state == 'filling'. Plans on 'wait_for_fill' but
    doesn't move.
  * Removed _maybe_finish_homing logic — homing is a separate operator step.
  * Added an initial "move_home" phase at the start of filling so the gantry
    starts at a known position before visiting holes.
"""

import json
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Point, PointStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy
from std_msgs.msg import Float32, Int32, String
# from std_msgs.msg import Float32, Int32, String


# Removed explicit latched QoS; use default QoS or simple depth args.


class FillingControl(Node):
    def __init__(self):
        super().__init__('filling_control')

        # ── Topic params ────────────────────────────────────────────────
        self.declare_parameter('result_topic',            '/tof_result')
        self.declare_parameter('main_state_topic',        '/main_state')
        self.declare_parameter('change_main_state_topic', '/change_main_state')
        self.declare_parameter('current_xy_topic',        '/current_xy_pos')
        # self.declare_parameter('current_z_topic',         '/current_z_pos')
        self.declare_parameter('target_xy_topic',         '/target_xy')
        # self.declare_parameter('target_z_topic',          '/target_z')
        self.declare_parameter('pump_cmd_topic',          '/pump_cmd')
        self.declare_parameter('status_topic',            'filling_status')
        self.declare_parameter('move_done_topic',         '/move_done')

        # ── Geometry / behaviour params ────────────────────────────────
        self.declare_parameter('position_scale_m',     0.001)
        self.declare_parameter('home_x_steps',         0.0)
        self.declare_parameter('home_y_steps',         0.0)
        # self.declare_parameter('home_z_m',             0.0)
        # self.declare_parameter('approach_clearance_m', 0.03)
        # self.declare_parameter('fill_z_offset_m',      0.0)
        self.declare_parameter('xy_tolerance_m',       0.003)
        # self.declare_parameter('z_tolerance_m',        0.003)
        self.declare_parameter('pump_rate_cm3_s',      1.0)
        self.declare_parameter('min_pump_ms',          200)
        self.declare_parameter('max_pump_ms',          60000)
        self.declare_parameter('move_settle_s',        0.2)

        # ── Per-phase timeouts ─────────────────────────────────────────
        self.declare_parameter('move_xy_timeout_s', 30.0)
        self.declare_parameter('lower_timeout_s',   20.0)
        self.declare_parameter('raise_timeout_s',   20.0)
        self.declare_parameter('home_timeout_s',    60.0)

        self.result_topic            = self.get_parameter('result_topic').value
        self.main_state_topic        = self.get_parameter('main_state_topic').value
        self.change_main_state_topic = self.get_parameter('change_main_state_topic').value
        self.current_xy_topic        = self.get_parameter('current_xy_topic').value
        # self.current_z_topic         = self.get_parameter('current_z_topic').value
        self.target_xy_topic         = self.get_parameter('target_xy_topic').value
        # self.target_z_topic          = self.get_parameter('target_z_topic').value
        self.pump_cmd_topic          = self.get_parameter('pump_cmd_topic').value
        self.status_topic            = self.get_parameter('status_topic').value
        self.move_done_topic         = self.get_parameter('move_done_topic').value

        self.position_scale_m     = float(self.get_parameter('position_scale_m').value)
        self.home_x_steps         = float(self.get_parameter('home_x_steps').value)
        self.home_y_steps         = float(self.get_parameter('home_y_steps').value)
        # self.home_z_m             = float(self.get_parameter('home_z_m').value)
        # self.approach_clearance_m = float(self.get_parameter('approach_clearance_m').value)
        # self.fill_z_offset_m      = float(self.get_parameter('fill_z_offset_m').value)
        self.xy_tolerance_m       = float(self.get_parameter('xy_tolerance_m').value)
        # self.z_tolerance_m        = float(self.get_parameter('z_tolerance_m').value)
        self.pump_rate_cm3_s      = float(self.get_parameter('pump_rate_cm3_s').value)
        self.min_pump_ms          = int(self.get_parameter('min_pump_ms').value)
        self.max_pump_ms          = int(self.get_parameter('max_pump_ms').value)
        self.move_settle_s        = float(self.get_parameter('move_settle_s').value)

        self.move_xy_timeout_s = float(self.get_parameter('move_xy_timeout_s').value)
        self.lower_timeout_s   = float(self.get_parameter('lower_timeout_s').value)
        self.raise_timeout_s   = float(self.get_parameter('raise_timeout_s').value)
        self.home_timeout_s    = float(self.get_parameter('home_timeout_s').value)

        # ── State ──────────────────────────────────────────────────────
        self.main_state         = 'free'
        self.result_ready       = False
        self.active             = False
        self.phase              = 'idle'
        self.phase_started_at: Optional[float] = None
        self.target_sent_for_phase = False

        self.current_hole_index = 0
        self.settle_until: Optional[float] = None
        self.pump_end_time: Optional[float] = None
        self._pump_sent_for_hole = False
        self._last_status: Optional[str] = None
        self._last_move_done_result: Optional[str] = None

        self.current_xy: Optional[Tuple[float, float]] = None
        # self.current_z: Optional[float] = None

        self.holes: List[Dict[str, float]] = []
        self.planned_holes: List[Dict[str, float]] = []

        # ── Subscribers ───────────────────────────────────────────────
        self.result_sub = self.create_subscription(
            String, self.result_topic, self.result_callback, 10)
        self.main_state_sub = self.create_subscription(
            String, self.main_state_topic, self.main_state_callback, 10)
        self.current_xy_sub = self.create_subscription(
            PointStamped, self.current_xy_topic, self.current_xy_callback, 10)
        # self.current_z_sub = self.create_subscription(
        #     Float32, self.current_z_topic, self.current_z_callback, 10)
        self.move_done_sub = self.create_subscription(
            String, self.move_done_topic, self.move_done_callback, 10)

        # ── Publishers ────────────────────────────────────────────────
        self.change_main_state_pub = self.create_publisher(
            String, self.change_main_state_topic, 10)
        self.target_xy_pub = self.create_publisher(Point,   self.target_xy_topic, 10)
        # self.target_z_pub  = self.create_publisher(Float32, self.target_z_topic,  10)
        self.pump_pub      = self.create_publisher(Int32,   self.pump_cmd_topic,  10)
        self.status_pub    = self.create_publisher(String,  self.status_topic,    10)

        self.timer = self.create_timer(0.05, self._tick)
        self._publish_status('waiting_for_result')

        self.get_logger().info(
            f'FillingControl started\n'
            f'  Operator-gated flow: result -> wait_for_fill -> (button) -> filling\n'
            f'  Tolerances    : xy={self.xy_tolerance_m*1000:.1f} mm  '
            f'  Z axis        : commented out\n'
            f'  Pump rate     : {self.pump_rate_cm3_s} cm^3/s'
        )

    # ──────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────
    def result_callback(self, msg: String):
        # Reject new results while we're actively filling
        if self.active:
            self.get_logger().warn('New /tof_result ignored - fill in progress')
            return

        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid JSON in result: {exc}')
            return

        holes: List[Dict[str, float]] = []
        for cluster in payload.get('clusters', []):
            centroid = cluster.get('centroid', {})
            x_m        = float(centroid.get('x', 0.0))
            y_m        = float(centroid.get('y', 0.0))
            z_m        = float(centroid.get('z', 0.0))
            volume_cm3 = float(cluster.get('volume_cm3', 0.0))
            if volume_cm3 <= 0.0:
                continue
            holes.append({
                'id':         float(cluster.get('id', len(holes) + 1)),
                'x_m':        x_m,
                'y_m':        y_m,
                'z_m':        z_m,
                'volume_cm3': volume_cm3,
                'pump_ms':    self._volume_to_pump_ms(volume_cm3),
            })

        self.holes = holes
        self.planned_holes = self._plan_holes(holes)
        self.result_ready  = True

        if self.planned_holes:
            self._publish_status(f'result_ready_{len(self.planned_holes)}_holes_awaiting_operator')
            total_vol = sum(h['volume_cm3'] for h in holes)
            self.get_logger().info(
                f'Result received: {len(self.planned_holes)} hole(s) planned. '
                f'Total volume: {total_vol:.2f} cm^3. '
                f"Waiting for operator to press 'Start Filling' button."
            )
        else:
            self._publish_status('no_holes')
            self.get_logger().info('Result received: no fillable holes.')

        # We do NOT call _maybe_start() here. Operator must press the button.

    def main_state_callback(self, msg: String):
        previous = self.main_state
        state = msg.data.strip() or 'free'
        self.main_state = state

        # External abort: if main_state goes to 'free' mid-fill, stop everything.
        if state == 'free' and self.active and previous != 'free':
            self.get_logger().warn('Abort requested via /main_state=free')
            self._abort('external_abort')
            return

        # Operator pressed the button: wait_for_fill -> filling
        if state == 'filling' and previous == 'wait_for_fill':
            self._maybe_start()
            return

        # If we somehow get filling directly (operator override etc.), still try
        if state == 'filling' and not self.active:
            self._maybe_start()

    def current_xy_callback(self, msg: PointStamped):
        self.current_xy = (
            float(msg.point.x) * self.position_scale_m,
            float(msg.point.y) * self.position_scale_m,
        )

    # def current_z_callback(self, msg: Float32):
    #     self.current_z = float(msg.data)

    def move_done_callback(self, msg: String):
        self._last_move_done_result = msg.data.strip()
        self.get_logger().info(f'XY move done: {self._last_move_done_result}')

    # ──────────────────────────────────────────────────────────────────
    # Start / abort
    # ──────────────────────────────────────────────────────────────────
    def _maybe_start(self):
        if self.active:
            return
        if not self.result_ready:
            self.get_logger().warn(
                "Got 'filling' state but no /tof_result received yet — ignoring."
            )
            return
        if not self.planned_holes:
            self.get_logger().info('Filling requested but no holes to fill — completing.')
            self._publish_status('complete_no_holes')
            self._publish_change_state('free')
            return

        self.active = True
        self.current_hole_index = 0
        self.settle_until = None
        self.pump_end_time = None
        self._pump_sent_for_hole = False

        # First step of filling: return to home so we start from a known pose.
        self._enter_phase('move_home')
        self._publish_status('filling_start_moving_home')

    def _abort(self, reason: str):
        # Emergency stop pump
        pump_msg = Int32()
        pump_msg.data = 0
        self.pump_pub.publish(pump_msg)

        self.active = False
        self.phase = 'idle'
        self.target_sent_for_phase = False
        self.settle_until = None
        self.pump_end_time = None
        self._pump_sent_for_hole = False

        self._publish_status(f'aborted_{reason}')

    # ──────────────────────────────────────────────────────────────────
    # Phase helpers
    # ──────────────────────────────────────────────────────────────────
    def _enter_phase(self, name: str):
        self.phase = name
        self.phase_started_at = self._now_sec()
        self.target_sent_for_phase = False

    def _phase_elapsed(self) -> float:
        if self.phase_started_at is None:
            return 0.0
        return self._now_sec() - self.phase_started_at

    def _phase_timeout(self, limit_s: float) -> bool:
        if self._phase_elapsed() > limit_s:
            self.get_logger().error(
                f'Phase {self.phase} timed out after {limit_s:.1f}s'
            )
            self._publish_status(f'timeout_{self.phase}')
            self._abort(f'timeout_{self.phase}')
            return True
        return False

    # ──────────────────────────────────────────────────────────────────
    # Tick — main state machine
    # ──────────────────────────────────────────────────────────────────
    def _tick(self):
        if not self.active:
            return

        # ── Pre-hole: move to home position once at start ─────────────
        if self.phase == 'move_home':
            home_x_m = self.home_x_steps * self.position_scale_m
            home_y_m = self.home_y_steps * self.position_scale_m

            if not self.target_sent_for_phase:
                self._publish_xy_target(home_x_m, home_y_m)
                # self._publish_z_target(self.home_z_m)
                self.target_sent_for_phase = True

            if self._phase_timeout(self.home_timeout_s):
                return
            if self._at_xy(home_x_m, home_y_m):
                # if self._at_xy(home_x_m, home_y_m) and self._at_z(self.home_z_m):
                self._enter_phase('move_xy_above')
                self._publish_status('at_home_starting_holes')
            return

        # ── Done? ─────────────────────────────────────────────────────
        if self.current_hole_index >= len(self.planned_holes):
            self.active = False
            self.phase = 'done'
            self._publish_status('complete')
            self._publish_change_state('free')
            return

        hole = self.planned_holes[self.current_hole_index]
        # approach_z = max(self.home_z_m, hole['z_m'] + self.approach_clearance_m)
        # fill_z     = hole['z_m'] + self.fill_z_offset_m

        hole_id = int(hole['id'])
        idx     = self.current_hole_index + 1
        total   = len(self.planned_holes)

        # ── MOVE XY + approach Z ──────────────────────────────────────
        if self.phase == 'move_xy_above':
            if not self.target_sent_for_phase:
                self._publish_xy_target(hole['x_m'], hole['y_m'])
                # self._publish_z_target(approach_z)
                self.target_sent_for_phase = True
                self._publish_status(f'move_above_{idx}/{total}_id{hole_id}')

            if self._phase_timeout(self.move_xy_timeout_s):
                return
            if self._at_xy(hole['x_m'], hole['y_m']):
                # if self._at_xy(hole['x_m'], hole['y_m']) and self._at_z(approach_z):
                self.settle_until = self._now_sec() + self.move_settle_s
                self._enter_phase('settle_above')
            return

        # ── Settle ────────────────────────────────────────────────────
        if self.phase == 'settle_above':
            if self.settle_until is None or self._now_sec() < self.settle_until:
                return
            # self._enter_phase('lower')
            # self._publish_status(f'lower_{idx}/{total}_id{hole_id}')
            self._enter_phase('pump')
            self._publish_status(f'pump_{idx}/{total}_id{hole_id}')
            return

        # ── Lower Z ───────────────────────────────────────────────────
        # if self.phase == 'lower':
        #     if not self.target_sent_for_phase:
        #         self._publish_z_target(fill_z)
        #         self.target_sent_for_phase = True

        #     if self._phase_timeout(self.lower_timeout_s):
        #         return
        #     if self._at_z(fill_z):
        #         self._enter_phase('pump')
        #         self._pump_sent_for_hole = False
        #     return

        # ── Pump ──────────────────────────────────────────────────────
        if self.phase == 'pump':
            if not self._pump_sent_for_hole:
                pump_msg = Int32()
                pump_msg.data = int(hole['pump_ms'])
                self.pump_pub.publish(pump_msg)
                self.pump_end_time = self._now_sec() + (hole['pump_ms'] / 1000.0)
                self._pump_sent_for_hole = True
                self._publish_status(
                    f'pump_{idx}/{total}_id{hole_id}_'
                    f'{int(hole["pump_ms"])}ms_'
                    f'{hole["volume_cm3"]:.2f}cm3'
                )
                return
            if self.pump_end_time is not None and self._now_sec() < self.pump_end_time:
                return
            # self._enter_phase('raise')
            # self._publish_status(f'raise_{idx}/{total}_id{hole_id}')
            self.current_hole_index += 1
            self._enter_phase('move_xy_above')
            self.settle_until = None
            self.pump_end_time = None
            self._pump_sent_for_hole = False

        # ── Raise ─────────────────────────────────────────────────────
        # if self.phase == 'raise':
        #     if not self.target_sent_for_phase:
        #         self._publish_z_target(approach_z)
        #         self.target_sent_for_phase = True

        #     if self._phase_timeout(self.raise_timeout_s):
        #         return
        #     if self._at_z(approach_z):
        #         self.current_hole_index += 1
        #         self._enter_phase('move_xy_above')
        #         self.settle_until = None
        #         self.pump_end_time = None
        #         self._pump_sent_for_hole = False

    # ──────────────────────────────────────────────────────────────────
    # Planning
    # ──────────────────────────────────────────────────────────────────
    def _plan_holes(self, holes: List[Dict[str, float]]) -> List[Dict[str, float]]:
        """Nearest-neighbour from home position (we'll be there when filling starts)."""
        remaining = holes[:]
        ordered: List[Dict[str, float]] = []
        current_x = self.home_x_steps * self.position_scale_m
        current_y = self.home_y_steps * self.position_scale_m

        while remaining:
            next_hole = min(
                remaining,
                key=lambda hole: (
                    (hole['x_m'] - current_x) ** 2
                    + (hole['y_m'] - current_y) ** 2
                ) ** 0.5,
            )
            ordered.append(next_hole)
            current_x = next_hole['x_m']
            current_y = next_hole['y_m']
            remaining.remove(next_hole)

        return ordered

    # ──────────────────────────────────────────────────────────────────
    # Publish helpers
    # ──────────────────────────────────────────────────────────────────
    def _publish_xy_target(self, x_m: float, y_m: float):
        target = Point()
        target.x = self._meters_to_xy_steps(x_m)
        target.y = self._meters_to_xy_steps(y_m)
        target.z = 0.0
        self.target_xy_pub.publish(target)
        self._last_move_done_result = None

    # def _publish_z_target(self, z_m: float):
    #     msg = Float32()
    #     msg.data = float(z_m)
    #     self.target_z_pub.publish(msg)

    def _publish_change_state(self, state: str):
        msg = String()
        msg.data = state
        self.change_main_state_pub.publish(msg)

    def _publish_status(self, text: str):
        if text == self._last_status:
            return
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self._last_status = text

    # ──────────────────────────────────────────────────────────────────
    # Math helpers
    # ──────────────────────────────────────────────────────────────────
    def _meters_to_xy_steps(self, meters: float) -> float:
        if self.position_scale_m <= 0.0:
            return meters
        return meters / self.position_scale_m

    def _volume_to_pump_ms(self, volume_cm3: float) -> int:
        if self.pump_rate_cm3_s <= 0.0:
            return self.min_pump_ms
        ms = int(round((volume_cm3 / self.pump_rate_cm3_s) * 1000.0))
        return max(self.min_pump_ms, min(self.max_pump_ms, ms))

    def _at_xy(self, x_m: float, y_m: float) -> bool:
        if self.current_xy is None:
            return False
        if self._last_move_done_result is not None:
            if self._last_move_done_result.startswith('ok'):
                return True
        return (abs(self.current_xy[0] - x_m) <= self.xy_tolerance_m
                and abs(self.current_xy[1] - y_m) <= self.xy_tolerance_m)

    # def _at_z(self, z_m: float) -> bool:
    #     if self.current_z is None:
    #         return False
    #     return abs(self.current_z - z_m) <= self.z_tolerance_m

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = FillingControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Filling control interrupted, shutting down.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()