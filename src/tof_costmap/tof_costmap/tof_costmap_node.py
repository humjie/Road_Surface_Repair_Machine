import math
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, Range


class TofCostmapNode(Node):
    def __init__(self):
        super().__init__('tof_costmap_node')

        self.declare_parameter('input_topic', 'tof_data')
        self.declare_parameter('output_topic', 'tof_costmap/points')
        self.declare_parameter('frame_id', 'tof_sensor_link')

        self.declare_parameter('x_speed_mm_s', 10.0)
        self.declare_parameter('y_speed_mm_s', 10.0)
        self.declare_parameter('x_length_mm', 100.0)
        self.declare_parameter('y_length_mm', 100.0)
        self.declare_parameter('scan_step_mm', 10.0)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        self.x_speed_mm_s = float(self.get_parameter('x_speed_mm_s').value)
        self.y_speed_mm_s = float(self.get_parameter('y_speed_mm_s').value)
        self.x_length_mm = float(self.get_parameter('x_length_mm').value)
        self.y_length_mm = float(self.get_parameter('y_length_mm').value)
        self.scan_step_mm = float(self.get_parameter('scan_step_mm').value)

        self.SCANNING_X = 'SCANNING_X'
        self.STEPPING_Y = 'STEPPING_Y'
        self.DONE = 'DONE'

        self.state = self.SCANNING_X
        self.current_x_mm = 0.0
        self.current_y_mm = 0.0
        self.direction_x = 1
        self.target_y_mm = 0.0
        self.last_grid_x = -1

        self.last_stamp_sec = None
        self.points_by_cell = {}

        self.subscription = self.create_subscription(
            Range,
            self.input_topic,
            self.range_callback,
            10,
        )
        self.publisher = self.create_publisher(PointCloud2, self.output_topic, 10)

        self.get_logger().info(
            f'Subscribed to {self.input_topic}; publishing 3D costmap PointCloud2 on {self.output_topic}'
        )

    def range_callback(self, msg: Range):
        if self.state == self.DONE:
            return

        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if self.last_stamp_sec is None:
            self.last_stamp_sec = stamp_sec
            dt = 0.0
        else:
            dt = max(0.0, stamp_sec - self.last_stamp_sec)
            self.last_stamp_sec = stamp_sec

        self.update_scan_state(dt)
        self.update_costmap(msg.range)
        self.publish_cloud(msg.header.stamp)

    def update_scan_state(self, dt: float):
        if self.state == self.SCANNING_X:
            self.current_x_mm += self.x_speed_mm_s * dt * self.direction_x

            if (self.direction_x == 1 and self.current_x_mm >= self.x_length_mm) or (
                self.direction_x == -1 and self.current_x_mm <= 0.0
            ):
                self.current_x_mm = self.x_length_mm if self.direction_x == 1 else 0.0
                self.target_y_mm = self.current_y_mm + self.scan_step_mm
                self.direction_x *= -1
                self.state = self.STEPPING_Y

        elif self.state == self.STEPPING_Y:
            self.current_y_mm += self.y_speed_mm_s * dt

            if self.current_y_mm >= self.target_y_mm:
                self.current_y_mm = self.target_y_mm
                if self.current_y_mm > self.y_length_mm:
                    self.state = self.DONE
                    self.get_logger().info('ToF scan complete; keeping final costmap published.')
                else:
                    self.last_grid_x = -1
                    self.state = self.SCANNING_X

    def update_costmap(self, distance_m: float):
        if self.state != self.SCANNING_X:
            return

        current_grid_x = int(round(self.current_x_mm / self.scan_step_mm))
        if current_grid_x == self.last_grid_x:
            return

        grid_y = int(round(self.current_y_mm / self.scan_step_mm))

        x_m = self.current_x_mm / 1000.0
        y_m = self.current_y_mm / 1000.0
        z_m = max(0.0, float(distance_m))

        cell_key = (current_grid_x, grid_y)
        self.points_by_cell[cell_key] = (x_m, y_m, z_m, z_m)
        self.last_grid_x = current_grid_x

    def publish_cloud(self, stamp):
        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = self.frame_id
        cloud.height = 1
        cloud.width = len(self.points_by_cell)
        cloud.is_bigendian = False
        cloud.is_dense = True

        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        cloud.point_step = 16
        cloud.row_step = cloud.point_step * cloud.width

        data = bytearray()
        for point in self.points_by_cell.values():
            data.extend(struct.pack('<ffff', point[0], point[1], point[2], point[3]))
        cloud.data = bytes(data)

        self.publisher.publish(cloud)


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