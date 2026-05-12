import json
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Point, PointStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from std_msgs.msg import Float32, Int32, String


LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class FillingControl(Node):
    def __init__(self):
        super().__init__('filling_control')

        self.declare_parameter('result_topic', '/tof_result')
        self.declare_parameter('main_state_topic', '/main_state')
        self.declare_parameter('change_main_state_topic', '/change_main_state')
        self.declare_parameter('current_xy_topic', '/current_xy_pos')
        self.declare_parameter('current_z_topic', '/current_z_pos')
        self.declare_parameter('target_xy_topic', '/target_xy')
        self.declare_parameter('target_z_topic', '/target_z')
        self.declare_parameter('pump_cmd_topic', '/pump_cmd')
        self.declare_parameter('status_topic', 'filling_status')

        self.declare_parameter('position_scale_m', 0.001)
        self.declare_parameter('home_xy_steps', 0.0)
        self.declare_parameter('home_z_m', 0.0)
        self.declare_parameter('approach_clearance_m', 0.03)
        self.declare_parameter('z_tolerance_m', 0.003)
        self.declare_parameter('pump_rate_cm3_s', 1.0)
        self.declare_parameter('min_pump_ms', 200)
        self.declare_parameter('max_pump_ms', 60000)
        self.declare_parameter('move_settle_s', 0.2)

        self.result_topic = self.get_parameter('result_topic').value
        self.main_state_topic = self.get_parameter('main_state_topic').value
        self.change_main_state_topic = self.get_parameter('change_main_state_topic').value
        self.current_xy_topic = self.get_parameter('current_xy_topic').value
        self.current_z_topic = self.get_parameter('current_z_topic').value
        self.target_xy_topic = self.get_parameter('target_xy_topic').value
        self.target_z_topic = self.get_parameter('target_z_topic').value
        self.pump_cmd_topic = self.get_parameter('pump_cmd_topic').value
        self.status_topic = self.get_parameter('status_topic').value

        self.position_scale_m = float(self.get_parameter('position_scale_m').value)
        self.home_xy_steps = float(self.get_parameter('home_xy_steps').value)
        self.home_z_m = float(self.get_parameter('home_z_m').value)
        self.approach_clearance_m = float(self.get_parameter('approach_clearance_m').value)
        self.z_tolerance_m = float(self.get_parameter('z_tolerance_m').value)
        self.pump_rate_cm3_s = float(self.get_parameter('pump_rate_cm3_s').value)
        self.min_pump_ms = int(self.get_parameter('min_pump_ms').value)
        self.max_pump_ms = int(self.get_parameter('max_pump_ms').value)
        self.move_settle_s = float(self.get_parameter('move_settle_s').value)

        self.main_state = 'free'
        self.result_ready = False
        self.homing_requested = False
        self.homed = False
        self.active = False
        self.phase = 'idle'
        self.current_hole_index = 0
        self.settle_until: Optional[float] = None
        self.pump_end_time: Optional[float] = None
        self._pump_sent_for_hole = False
        self._last_status: Optional[str] = None

        self.current_xy: Optional[Tuple[float, float]] = None
        self.current_z: Optional[float] = None

        self.holes: List[Dict[str, float]] = []
        self.planned_holes: List[Dict[str, float]] = []

        self.result_sub = self.create_subscription(
            String, self.result_topic, self.result_callback, LATCHED_QOS)
        self.main_state_sub = self.create_subscription(
            String, self.main_state_topic, self.main_state_callback, 10)
        self.current_xy_sub = self.create_subscription(
            PointStamped, self.current_xy_topic, self.current_xy_callback, 10)
        self.current_z_sub = self.create_subscription(
            Float32, self.current_z_topic, self.current_z_callback, 10)

        self.change_main_state_pub = self.create_publisher(String, self.change_main_state_topic, 10)
        self.target_xy_pub = self.create_publisher(Point, self.target_xy_topic, 10)
        self.target_z_pub = self.create_publisher(Float32, self.target_z_topic, 10)
        self.pump_pub = self.create_publisher(Int32, self.pump_cmd_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.timer = self.create_timer(0.05, self._tick)
        self._publish_status('waiting_for_result')

        self.get_logger().info(
            f'FillingControl started\n'
            f'  Result topic      : {self.result_topic}\n'
            f'  Main state topic  : {self.main_state_topic}\n'
            f'  Change state topic: {self.change_main_state_topic}\n'
            f'  Target XY topic   : {self.target_xy_topic}\n'
            f'  Target Z topic    : {self.target_z_topic}\n'
            f'  Pump command      : {self.pump_cmd_topic}\n'
            f'  Status topic      : {self.status_topic}'
        )

    def result_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid JSON in result: {exc}')
            return

        holes: List[Dict[str, float]] = []
        for cluster in payload.get('clusters', []):
            centroid = cluster.get('centroid', {})
            x_m = float(centroid.get('x', 0.0))
            y_m = float(centroid.get('y', 0.0))
            z_m = float(centroid.get('z', 0.0))
            volume_cm3 = float(cluster.get('volume_cm3', 0.0))
            if volume_cm3 <= 0.0:
                continue
            holes.append({
                'id': float(cluster.get('id', len(holes) + 1)),
                'x_m': x_m,
                'y_m': y_m,
                'z_m': z_m,
                'volume_cm3': volume_cm3,
                'pump_ms': self._volume_to_pump_ms(volume_cm3),
            })

        self.holes = holes
        self.planned_holes = self._plan_holes(holes)
        self.result_ready = True

        if self.planned_holes:
            self._publish_status(f'result_ready_{len(self.planned_holes)}')
        else:
            self._publish_status('no_holes')

        self._maybe_start()

    def main_state_callback(self, msg: String):
        state = msg.data.strip() or 'free'
        self.main_state = state

        if state == 'filling':
            self._maybe_start()
        elif state == 'free' and self.homing_requested and not self.homed:
            self._maybe_finish_homing()

    def current_xy_callback(self, msg: PointStamped):
        self.current_xy = (
            float(msg.point.x) * self.position_scale_m,
            float(msg.point.y) * self.position_scale_m,
        )

    def current_z_callback(self, msg: Float32):
        self.current_z = float(msg.data)

    def _maybe_start(self):
        if not self.result_ready or not self.planned_holes:
            return

        if not self.homed:
            if not self.homing_requested:
                self._request_homing()
            return

        if self.main_state != 'filling' or self.active:
            return

        self.active = True
        self.phase = 'move_xy_above'
        self.current_hole_index = 0
        self.settle_until = None
        self.pump_end_time = None
        self._pump_sent_for_hole = False
        self._publish_status('filling_start')

    def _request_homing(self):
        self.homing_requested = True
        self.phase = 'waiting_homing'
        self._publish_status('request_homing')
        self._publish_change_state('homing')

    def _maybe_finish_homing(self):
        if self.current_xy is None or self.current_z is None:
            return
        if not self._at_home():
            return

        self.homed = True
        self.homing_requested = False
        self._publish_status('homed')
        self._publish_change_state('filling')

    def _tick(self):
        if not self.active:
            return

        if self.current_hole_index >= len(self.planned_holes):
            self.active = False
            self.phase = 'done'
            self._publish_status('complete')
            self._publish_change_state('free')
            return

        hole = self.planned_holes[self.current_hole_index]
        approach_z = max(self.home_z_m, hole['z_m'] + self.approach_clearance_m)
        fill_z = hole['z_m']

        if self.phase == 'move_xy_above':
            self._publish_xy_target(hole['x_m'], hole['y_m'])
            self._publish_z_target(approach_z)
            if self._at_xy(hole['x_m'], hole['y_m']) and self._at_z(approach_z):
                self.settle_until = self._now_sec() + self.move_settle_s
                self.phase = 'settle_above'
                self._publish_status(f'move_above_{self.current_hole_index + 1}')
            return

        if self.phase == 'settle_above':
            if self.settle_until is None or self._now_sec() < self.settle_until:
                return
            self.phase = 'lower'
            self._publish_status(f'lower_{self.current_hole_index + 1}')
            return

        if self.phase == 'lower':
            self._publish_z_target(fill_z)
            if self._at_z(fill_z):
                self.phase = 'pump'
                self._pump_sent_for_hole = False
            return

        if self.phase == 'pump':
            if not self._pump_sent_for_hole:
                pump_msg = Int32()
                pump_msg.data = int(hole['pump_ms'])
                self.pump_pub.publish(pump_msg)
                self.pump_end_time = self._now_sec() + (hole['pump_ms'] / 1000.0)
                self._pump_sent_for_hole = True
                self._publish_status(f'pump_{self.current_hole_index + 1}')
                return
            if self.pump_end_time is not None and self._now_sec() < self.pump_end_time:
                return
            self.phase = 'raise'
            self._publish_status(f'raise_{self.current_hole_index + 1}')
            return

        if self.phase == 'raise':
            self._publish_z_target(approach_z)
            if self._at_z(approach_z):
                self.current_hole_index += 1
                self.phase = 'move_xy_above'
                self.settle_until = None
                self.pump_end_time = None
                self._pump_sent_for_hole = False

    def _plan_holes(self, holes: List[Dict[str, float]]) -> List[Dict[str, float]]:
        remaining = holes[:]
        ordered: List[Dict[str, float]] = []
        current_x = self.home_xy_steps * self.position_scale_m
        current_y = self.home_xy_steps * self.position_scale_m

        while remaining:
            next_hole = min(
                remaining,
                key=lambda hole: ((hole['x_m'] - current_x) ** 2 + (hole['y_m'] - current_y) ** 2) ** 0.5,
            )
            ordered.append(next_hole)
            current_x = next_hole['x_m']
            current_y = next_hole['y_m']
            remaining.remove(next_hole)

        return ordered

    def _publish_xy_target(self, x_m: float, y_m: float):
        target = Point()
        target.x = self._meters_to_xy_steps(x_m)
        target.y = self._meters_to_xy_steps(y_m)
        target.z = 0.0
        self.target_xy_pub.publish(target)

    def _publish_z_target(self, z_m: float):
        msg = Float32()
        msg.data = float(z_m)
        self.target_z_pub.publish(msg)

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
        return abs(self.current_xy[0] - x_m) <= self.position_scale_m and abs(self.current_xy[1] - y_m) <= self.position_scale_m

    def _at_z(self, z_m: float) -> bool:
        if self.current_z is None:
            return False
        return abs(self.current_z - z_m) <= self.z_tolerance_m

    def _at_home(self) -> bool:
        home_xy_m = self.home_xy_steps * self.position_scale_m
        return self._at_xy(home_xy_m, home_xy_m) and self._at_z(self.home_z_m)

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
