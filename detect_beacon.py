import rclpy
from rclpy.node import Node
import imutils
import numpy as np
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from simple_pid import PID
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy


class DetectBeacon(Node):
    def __init__(self):
        # Initialise subs, pubs, service calls, path object
        super().__init__('detect_beacon')
        self.get_logger().info('Starting Detect Beacon Node')
        qos_policy = QoSProfile(durability=QoSDurabilityPolicy.VOLATILE, reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=5)

        self.subscriber_colour = self.create_subscription(Image, '/image_raw', self.callback, qos_policy)
        #self.subscriber_depth = self.create_subscription(Image, '/intel_realsense_r200_depth/depth/image_raw', self.callback_depth, qos_policy)
        self.subscriber_camera_info = self.create_subscription(CameraInfo, '/camera_info', self.callback_cam_info, qos_policy)
        #self.move_pub = self.create_publisher(Twist, '/cmd_vel', 1)

        self.bridge = CvBridge()
        self.colour_frame = None
        self.depth_frame = None
        self.num_colour_images = 0
        self.num_depth_images = 0
        self.K = None

        self.obstacle_detected = False
        self.user_in_frame = False
        self.park_mode = True

        #while rclpy.ok():
        #    self.callback_mode()
        #    rclpy.spin_once(self, timeout_sec=3.0) 


    def callback(self, colour_image):
        # to find user. green on top, pink on bottom
        self.get_logger().info('Now starting image processing.')
        if self.obstacle_detected == True: # if obstacle detected, don't bother going through image processing
            return
        self.colour_frame = self.bridge.imgmsg_to_cv2(colour_image, "bgr8")
        blurred = cv2.GaussianBlur(self.colour_frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        green_lower = (63, 200, 80)
        green_upper = (78, 255, 255)
        pink_lower = (148, 200, 80)
        pink_upper = (163, 255, 255)

        mask_green = cv2.inRange(hsv, green_lower, green_upper)
        mask_green = cv2.erode(mask_green, None, iterations=2)
        mask_green = cv2.dilate(mask_green, None, iterations=2)
        contours = cv2.findContours(mask_green.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 
        contours = imutils.grab_contours(contours)
        if len(contours) != 0:
            largest_contour = max(contours, key=cv2.contourArea)
            x_g, y_g, w_g, h_g = cv2.boundingRect(largest_contour)
            self.get_logger().info(f'green len(contours) = {len(contours)}')
            self.get_logger().info(f'x_g = {x_g}, y_g = {y_g}, w_g = {w_g}, h_g = {h_g}')

            if len(contours) >= 2: #if it sees 2 or more patches of this green, which it should if it sees the stripe
                i=0
                while i <= len(contours):
                    if contours[i].all() == largest_contour.all():
                        cont = list(contours)
                        cont.pop(i)
                        contours = tuple(cont)
                        break
                    i+=1
                sec_lrg_contour = max(contours, key=cv2.contourArea)
                x_g2, y_g2, w_g2, h_g2 = cv2.boundingRect(sec_lrg_contour)
                if abs(y_g - y_g2) < 5: #if largest and second largest contours are (almost) in line, its the green we want
                    x_g_mid = (x_g + (0.5*w_g) + x_g2 + (0.5*w_g2))/2 #avg of mid of both contours
                    y_g_mid = (y_g + (0.5*h_g) + y_g2 + (0.5*h_g2))/2
                    self.colour_frame = cv2.rectangle(self.colour_frame, (min(x_g, x_g2), min(y_g, y_g2)), (max((x_g+w_g), (x_g2+w_g2)), max((y_g+h_g), (y_g2+h_g2))), (200, 20, 0), 2)       
            else:
                x_g_mid = x_g + 0.5*w_g
                y_g_mid = y_g + 0.5*h_g
                self.colour_frame = cv2.rectangle(self.colour_frame, (x_g, y_g), (x_g+w_g, y_g+h_g), (200, 20, 0), 2)
            
            self.get_logger().info('Seeing green.')
            print('See green')
        else:
            x_g = y_g = w_g = h_g = sec_lrg_contour = x_g2 = y_g2 = w_g2 = h_g2 = x_g_mid = y_g_mid = 0
   

    def callback_cam_info(self, camera_info):
        self.K = np.array(camera_info.k).reshape([3,3])

              

# Main function
def main(args=None):
    rclpy.init(args=args)

    db = DetectBeacon()

    rclpy.spin(db)

if __name__ == '__main__':
    main()


