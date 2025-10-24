import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

class TFLunaNode(Node):
    def __init__(self):
        super().__init__('tf_luna')
        self.ser = serial.Serial('/dev/serial0', 115200, timeout=1)
        self.pub = self.create_publisher(Range, 'tf_luna/range', 10)
        self.timer = self.create_timer(0.1, self.read_lidar)

    def read_lidar(self):
        if self.ser.read() == b'Y':
            if self.ser.read() == b'Y':
                low = self.ser.read()
                high = self.ser.read()
                dist = (ord(high) << 8) + ord(low)
                _ = self.ser.read(5)  # discard rest of frame
                msg = Range()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.range = dist / 100.0  # convert cm → m
                self.pub.publish(msg)

def main():
    rclpy.init()
    node = TFLunaNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
