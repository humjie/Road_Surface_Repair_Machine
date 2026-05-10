import json
from typing import List, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Point


LATCHED_QOS = QoSProfile(
	reliability=ReliabilityPolicy.RELIABLE,
	durability=DurabilityPolicy.TRANSIENT_LOCAL,
	history=HistoryPolicy.KEEP_LAST,
	depth=1,
)


class FillingControl(Node):
	def __init__(self):
		super().__init__('filling_control')

		# Topics
		self.declare_parameter('result_topic', 'tof_costmap/result')
		self.declare_parameter('start_repair_topic', 'start_repair')
		self.declare_parameter('target_xy_topic', 'target_xy')
		self.declare_parameter('stepper_state_topic', 'stepper_state')
		self.declare_parameter('pump_cmd_topic', 'pump_cmd')
		self.declare_parameter('status_topic', 'filling_status')

		# Units and timing
		self.declare_parameter('position_scale_m', 0.001)
		self.declare_parameter('pump_rate_cm3_s', 1.0)
		self.declare_parameter('min_pump_ms', 200)
		self.declare_parameter('max_pump_ms', 60000)
		self.declare_parameter('min_move_wait_s', 0.2)
		self.declare_parameter('settle_time_s', 0.2)

		self.result_topic = self.get_parameter('result_topic').value
		self.start_repair_topic = self.get_parameter('start_repair_topic').value
		self.target_xy_topic = self.get_parameter('target_xy_topic').value
		self.stepper_state_topic = self.get_parameter('stepper_state_topic').value
		self.pump_cmd_topic = self.get_parameter('pump_cmd_topic').value
		self.status_topic = self.get_parameter('status_topic').value

		self.position_scale_m = float(self.get_parameter('position_scale_m').value)
		self.pump_rate_cm3_s = float(self.get_parameter('pump_rate_cm3_s').value)
		self.min_pump_ms = int(self.get_parameter('min_pump_ms').value)
		self.max_pump_ms = int(self.get_parameter('max_pump_ms').value)
		self.min_move_wait_s = float(self.get_parameter('min_move_wait_s').value)
		self.settle_time_s = float(self.get_parameter('settle_time_s').value)

		# State
		self.holes: List[Dict[str, float]] = []
		self.holes_ready = False
		self.start_requested = False
		self.active = False
		self.state = 'IDLE'
		self.current_index = 0
		self.stepper_state = 'available'
		self.move_started_time: Optional[float] = None
		self.settle_until: Optional[float] = None
		self.pump_end_time: Optional[float] = None
		self._last_status: Optional[str] = None

		# ROS I/O
		self.result_sub = self.create_subscription(
			String, self.result_topic, self.result_callback, LATCHED_QOS)
		self.start_sub = self.create_subscription(
			String, self.start_repair_topic, self.start_repair_callback, 10)
		self.stepper_state_sub = self.create_subscription(
			String, self.stepper_state_topic, self.stepper_state_callback, 10)

		self.target_pub = self.create_publisher(Point, self.target_xy_topic, 10)
		self.pump_pub = self.create_publisher(Int32, self.pump_cmd_topic, 10)
		self.status_pub = self.create_publisher(String, self.status_topic, 10)

		self.timer = self.create_timer(0.05, self._tick)
		self._publish_status('waiting_for_result')

		self.get_logger().info(
			f'FillingControl started\n'
			f'  Result topic      : {self.result_topic}\n'
			f'  Start repair topic: {self.start_repair_topic}\n'
			f'  Target XY topic   : {self.target_xy_topic}\n'
			f'  Stepper state     : {self.stepper_state_topic}\n'
			f'  Pump command      : {self.pump_cmd_topic}\n'
			f'  Status topic      : {self.status_topic}'
		)

	def result_callback(self, msg: String):
		if self.active:
			self.get_logger().warn('Ignoring new result while filling is active.')
			return
		try:
			payload = json.loads(msg.data)
		except json.JSONDecodeError as exc:
			self.get_logger().error(f'Invalid JSON in result: {exc}')
			return

		clusters = payload.get('clusters', [])
		holes: List[Dict[str, float]] = []

		for cluster in clusters:
			centroid = cluster.get('centroid', {})
			x_m = float(centroid.get('x', 0.0))
			y_m = float(centroid.get('y', 0.0))
			volume_cm3 = float(cluster.get('volume_cm3', 0.0))
			if volume_cm3 <= 0.0:
				continue
			pump_ms = self._volume_to_pump_ms(volume_cm3)
			holes.append({
				'x_m': x_m,
				'y_m': y_m,
				'volume_cm3': volume_cm3,
				'pump_ms': pump_ms,
			})

		self.holes = holes
		self.holes_ready = True

		if self.holes:
			self._publish_status('holes_ready')
		else:
			self._publish_status('no_holes')

		self._maybe_start()

	def start_repair_callback(self, msg: String):
		cmd = msg.data.strip().lower()
		if cmd != 'yes':
			return
		self.start_requested = True
		if not self.holes_ready:
			self._publish_status('waiting_for_result')
		self._maybe_start()

	def stepper_state_callback(self, msg: String):
		self.stepper_state = msg.data.strip() or 'available'

	def _maybe_start(self):
		if self.active:
			return
		if not (self.holes_ready and self.start_requested):
			return
		if not self.holes:
			self._publish_status('no_holes')
			return
		self.active = True
		self.state = 'MOVE'
		self.current_index = 0
		self.move_started_time = None
		self.pump_end_time = None
		self.settle_until = None
		self._publish_status('filling_start')

	def _tick(self):
		if not self.active:
			return

		if self.state == 'MOVE':
			hole = self.holes[self.current_index]
			target = Point()
			target.x = self._meters_to_steps(hole['x_m'])
			target.y = self._meters_to_steps(hole['y_m'])
			target.z = 0.0
			self.target_pub.publish(target)
			self.move_started_time = self._now_sec()
			self.settle_until = None
			self.state = 'WAIT_MOVE'
			self._publish_status(
				f'moving {self.current_index + 1}/{len(self.holes)}')
			return

		if self.state == 'WAIT_MOVE':
			if self.move_started_time is None:
				return
			elapsed = self._now_sec() - self.move_started_time

			if self.stepper_state == 'moving':
				return

			if elapsed < self.min_move_wait_s:
				return

			if self.stepper_state not in ('available', ''):
				return

			if self.settle_until is None:
				self.settle_until = self._now_sec() + self.settle_time_s
				return
			if self._now_sec() < self.settle_until:
				return

			hole = self.holes[self.current_index]
			pump_ms = int(hole['pump_ms'])
			pump_msg = Int32()
			pump_msg.data = pump_ms
			self.pump_pub.publish(pump_msg)
			self.pump_end_time = self._now_sec() + (pump_ms / 1000.0)
			self.state = 'PUMPING'
			self._publish_status(
				f'pumping {self.current_index + 1}/{len(self.holes)}')
			return

		if self.state == 'PUMPING':
			if self.pump_end_time is None:
				return
			if self._now_sec() < self.pump_end_time:
				return
			self.current_index += 1
			if self.current_index >= len(self.holes):
				self.active = False
				self.state = 'DONE'
				self._publish_status('complete')
			else:
				self.state = 'MOVE'

	def _volume_to_pump_ms(self, volume_cm3: float) -> int:
		if self.pump_rate_cm3_s <= 0.0:
			return self.min_pump_ms
		seconds = volume_cm3 / self.pump_rate_cm3_s
		ms = int(round(seconds * 1000.0))
		ms = max(self.min_pump_ms, ms)
		ms = min(self.max_pump_ms, ms)
		return ms

	def _meters_to_steps(self, meters: float) -> float:
		if self.position_scale_m <= 0.0:
			return meters
		return meters / self.position_scale_m

	def _publish_status(self, text: str):
		if text == self._last_status:
			return
		msg = String()
		msg.data = text
		self.status_pub.publish(msg)
		self._last_status = text

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
