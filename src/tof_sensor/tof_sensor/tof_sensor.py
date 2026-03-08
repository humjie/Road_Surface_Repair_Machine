import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Or Float32 if you only send numbers
import serial

class SimpleSerialPublisher(Node):
    def __init__(self):
        super().__init__('tof_serial_node')
        
        # Configuration
        self.port = '/dev/ttyUSB0'
        self.baud = 115200
        self.ser = serial.Serial(self.port, self.baud, timeout=1.0)
        
        # Publisher
        self.publisher_ = self.create_publisher(String, 'tof_data', 10)
        
        # Timer to poll serial
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info(f"Started reading from {self.port}")

    def timer_callback(self):
        if self.ser.in_waiting > 0:
            try:
                # Read line from Arduino
                line = self.ser.readline().decode('utf-8').rstrip()
                
                if line:
                    msg = String()
                    # Get ROS2 Time
                    now = self.get_clock().now().to_msg()
                    
                    # Combine timestamp and data into the string for easy viewing
                    # Format: [Seconds.Nanoseconds] Data
                    msg.data = f"[{now.sec}.{now.nanosec}] {line}"
                    
                    self.publisher_.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Serial Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSerialPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()