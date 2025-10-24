import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

class TFLunaNode(Node):
    def __init__(self):
        super().__init__('tf_luna')
        # Open serial port
        self.ser = serial.Serial('/dev/serial0', 115200, timeout=1)

        # Create publisher
        self.pub = self.create_publisher(Range, 'tf_luna/range', 10)

        # Create a template Range message with constant metadata
        self.msg = Range()
        self.msg.radiation_type = Range.INFRARED  # 1 = Infrared (TF-Luna)
        self.msg.field_of_view = 0.05             # ~3 degrees
        self.msg.min_range = 0.2                  # 20 cm
        self.msg.max_range = 8.0                  # 8 meters
        self.msg.header.frame_id = "tf_luna_link" # frame name for RViz, etc.

        # Run callback at 10 Hz
        self.timer = self.create_timer(0.1, self.read_lidar)

    def read_lidar(self):
        # Check for the start bytes 0x59 0x59 ('Y''Y')
        if self.ser.read() == b'Y':
            if self.ser.read() == b'Y':
                low = self.ser.read()
                high = self.ser.read()
                dist = (ord(high) << 8) + ord(low)
                _ = self.ser.read(5)  # discard rest of frame

                # Update only dynamic fields
                self.msg.header.stamp = self.get_clock().now().to_msg()
                self.msg.range = dist / 100.0  # convert cm → m

                self.pub.publish(self.msg)

def main():
    rclpy.init()
    node = TFLunaNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
