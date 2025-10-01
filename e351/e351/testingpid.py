import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from simple_pid import PID
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

class LaneFollower(Node):
    def __init__(self):
        super().__init__('lane_follower')
        self.get_logger().info('Starting Lane Follower Module.')

        qos_policy = QoSProfile(durability=QoSDurabilityPolicy.VOLATILE, reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=5)

        self.subscriber_colour = self.create_subscription(Image, '/intel_realsense_r200_depth/image_raw', self.callback_camera, qos_policy)

        self.bridge = CvBridge()



    def callback_camera(self, colour_msg):
        self.get_logger().info('Processing Camera Callback')
        colour_image = self.bridge.imgmsg_to_cv2(colour_msg,desired_encoding='bgr8')

        h, w, d = colour_image.shape
        self.get_logger().info(f'h = {h}, w = {w}, d = {d}')


def main(args=None):
    rclpy.init(args=args)
    lf = LaneFollower()
    rclpy.spin(lf)

if __name__ == '__main__':
    main()
