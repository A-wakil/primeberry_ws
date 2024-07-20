# ~/ros2_ws/src/encoder_reader/encoder_reader/encoder_reader_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray
import serial

class SerialReaderNode(Node):
    def __init__(self):
        super().__init__('encoder_reader_node')
        self.publisher_ = self.create_publisher(Int64MultiArray, 'encoder_values', 10)
        self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if self.serial_port.in_waiting > 0:
            try:
                line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                self.get_logger().info(f'Raw data: {line}')
                encoder1_value, encoder2_value = map(int, line.split(","))
                msg = Int64MultiArray()
                msg.data = [-encoder1_value, -encoder2_value]
                self.publisher_.publish(msg)
            except ValueError:
                self.get_logger().error(f'Invalid data received: {line}')
            except UnicodeDecodeError as e:
                self.get_logger().error(f'UnicodeDecodeError: {e}')
            except Exception as e:
                self.get_logger().error(f'Unexpected error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
