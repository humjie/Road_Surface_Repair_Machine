from rcl_interfaces import msg
import rclpy
from rclpy.node import Node
import serial
import time
from sensor_msgs.msg import Range

class TofPublisher(Node):

    def __init__(self):
        super().__init__('tof_publisher')
        
        # Initialize serial connection
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        time.sleep(2)  # Wait for the serial connection to initialize
        
        self.publisher_ = self.create_publisher(Range, 'tof_data', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if self.ser.in_waiting > 0:
            
            # Read the line and decode it. Use errors='ignore' to handle corrupted bytes safely.
            raw_line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            
            # If the line is empty after stripping whitespace, skip this iteration
            if not raw_line:
                return

            # Safely attempt to convert the string to a float
            try:
                distance_mm = float(raw_line)
            except ValueError:
                self.get_logger().warn(f"Received invalid or incomplete data from sensor: '{raw_line}'")
                return

            # Construct and publish the message
            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'tof_sensor_link'
            msg.radiation_type = Range.INFRARED # ToF uses IR lasers
            msg.field_of_view = 0.436           # Approx 25 degrees in radians (VL53L0X default)
            msg.min_range = 0.02                # 2 cm minimum
            msg.max_range = 4.00                # 400 cm maximum

            # Convert mm to meters as required by the sensor_msgs/Range specification
            msg.range = distance_mm / 1000.0

            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.range)

            # Optional: If your sensor publishes data much faster than 0.5s (2Hz) 
            # and you want to prevent lag/backlog, flush the buffer *after* reading a valid line.
            self.ser.reset_input_buffer()

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    tof_publisher = TofPublisher()
    
    try:
        rclpy.spin(tof_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        tof_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()