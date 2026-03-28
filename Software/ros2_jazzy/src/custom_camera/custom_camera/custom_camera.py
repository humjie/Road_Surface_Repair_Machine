#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # Create publisher for Foxglove
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        
        # Open USB cam
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera")
            raise RuntimeError("Failed to open camera")
            
        self.get_logger().info("Camera opened. Continuously publishing to /camera/image_raw...")
        self.get_logger().info("Press Ctrl+C to stop.")

        # Set a timer to capture and publish frames at ~30 FPS (0.033 seconds)
        self.timer = self.create_timer(0.033, self.timer_callback)

    def timer_callback(self):
        ok, frame = self.cap.read()
        if ok:
            # Convert OpenCV frame to ROS 2 Image message and publish
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)
        else:
            self.get_logger().warning("Can't receive frame")

    def destroy_node(self):
        # Clean up the camera connection when the node is destroyed
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()

    try:
        # rclpy.spin() keeps the node running and handles the timer callbacks automatically
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("\nKeyboard Interrupt (Ctrl+C) detected. Shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()