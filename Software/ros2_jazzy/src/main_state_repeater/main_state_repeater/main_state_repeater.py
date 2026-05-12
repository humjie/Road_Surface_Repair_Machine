#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MainStateRepeater(Node):
    def __init__(self):
        super().__init__('main_state_repeater')

        self.declare_parameter('initial_state', 'free')
        self.declare_parameter('main_state_topic', '/main_state')
        self.declare_parameter('change_main_state_topic', '/change_main_state')

        self.current_state = str(self.get_parameter('initial_state').value).strip() or 'free'
        self.main_state_topic = self.get_parameter('main_state_topic').value
        self.change_main_state_topic = self.get_parameter('change_main_state_topic').value

        self.state_pub = self.create_publisher(String, self.main_state_topic, 10)
        self.state_sub = self.create_subscription(
            String, self.change_main_state_topic, self.state_change_callback, 10)
        self.timer = self.create_timer(0.1, self.publish_state)

        self.get_logger().info(
            f'MainStateRepeater initialized with state: {self.current_state}'
        )

    def publish_state(self):
        msg = String()
        msg.data = self.current_state
        self.state_pub.publish(msg)

    def state_change_callback(self, msg: String):
        requested_state = msg.data.strip()
        if not requested_state:
            return

        if requested_state == self.current_state:
            return

        if self.current_state != 'free' and requested_state not in ('free', self.current_state):
            self.get_logger().warn(
                f'Rejecting state change to {requested_state} while in {self.current_state}'
            )
            return

        previous_state = self.current_state
        self.current_state = requested_state
        self.get_logger().info(
            f'State changed from {previous_state} to {self.current_state}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MainStateRepeater()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
