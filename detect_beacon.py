import rclpy
from rclpy.node import Node
import imutils
import numpy as np
import cv2
import RPi.GPIO as GPIO
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
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
        self.button_latch = False

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(16,GPIO.OUT) # park mode LED
        GPIO.setup(4,GPIO.OUT) # follower mode LED
        GPIO.setup(12,GPIO.OUT) # user seen LED
        GPIO.setup(22,GPIO.IN) # mode changer button

        self.timer = self.create_timer(0.1, self.callback_mode)

        #while rclpy.ok():
        #    self.callback_mode()
        #    rclpy.spin_once(self, timeout_sec=3.0) 


    def callback_mode(self):
        button_on = GPIO.input(22)

        if self.park_mode == True:
            GPIO.output(4,GPIO.LOW)
            GPIO.output(16,GPIO.HIGH)
        else:
            GPIO.output(16,GPIO.LOW)
            GPIO.output(4,GPIO.HIGH)
        
        if button_on == 1 and self.button_latch == False: # change mode button
            if self.park_mode == True:
                self.park_mode = False
                self.get_logger().info('Now in follower mode.')
            else:
                self.park_mode = True
                self.get_logger().info('Now in park mode.')
            self.button_latch = True
        elif button_on == 0 and self.button_latch == True:
            self.button_latch = False

    def callback(self, colour_image):
        # to find user. green on top, pink on bottom
        if self.park_mode == True: # if obstacle detected, don't bother going through image processing
            return
        self.get_logger().info('Now starting image processing.')
        self.colour_frame = self.bridge.imgmsg_to_cv2(colour_image, "bgr8")
        blurred = cv2.GaussianBlur(self.colour_frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        green_lower = (48, 150, 80)
        green_upper = (58, 255, 255)
        pink_lower = (160, 150, 80)
        pink_upper = (170, 255, 255)

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
                if abs(y_g - y_g2) < 10: #if largest and second largest contours are (almost) in line, its the green we want
                    x_g_mid = (x_g + (0.5*w_g) + x_g2 + (0.5*w_g2))/2 #avg of mid of both contours
                    y_g_mid = (y_g + (0.5*h_g) + y_g2 + (0.5*h_g2))/2
                    self.colour_frame = cv2.rectangle(self.colour_frame, (min(x_g, x_g2), min(y_g, y_g2)), (max((x_g+w_g), (x_g2+w_g2)), max((y_g+h_g), (y_g2+h_g2))), (200, 20, 0), 2) 
                    self.get_logger().info('greens are level')
                else:
                    x_g_mid = 0
                    y_g_mid = 0
            else:
                x_g_mid = 0
                y_g_mid = 0
            
            self.get_logger().info('Seeing green.')
        else:
            x_g = y_g = w_g = h_g = sec_lrg_contour = x_g2 = y_g2 = w_g2 = h_g2 = x_g_mid = y_g_mid = 0

        mask_pink = cv2.inRange(hsv, pink_lower, pink_upper)
        mask_pink = cv2.erode(mask_pink, None, iterations=2)
        mask_pink = cv2.dilate(mask_pink, None, iterations=2)
        contours = cv2.findContours(mask_pink.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 
        contours = imutils.grab_contours(contours)
        if len(contours) != 0:
            largest_contour = max(contours, key=cv2.contourArea)
            x_p, y_p, w_p, h_p = cv2.boundingRect(largest_contour)
            self.get_logger().info(f'pink len(contours) = {len(contours)}')
            self.get_logger().info(f'x_p = {x_p}, y_p = {y_p}, w_p = {w_p}, h_p = {h_p}')

            if len(contours) >=2:
                i=0
                while i <= len(contours):
                    if contours[i].all() == largest_contour.all(): # error on this line, bool has no attribute all
                        cont = list(contours)
                        cont.pop(i)
                        contours = tuple(cont)
                        break
                    i+=1
                sec_lrg_contour = max(contours, key=cv2.contourArea)
                x_p2, y_p2, w_p2, h_p2 = cv2.boundingRect(sec_lrg_contour)
                self.get_logger().info(f'x_p2 = {x_p2}, y_p2 = {y_p2}, w_p2 = {w_p2}, h_p2 = {h_p2}')
                if abs(y_p - y_p2) < 10:
                    self.get_logger().info('pinks are level')
                    x_p_mid = (x_p + (0.5*w_p) + x_p2 + (0.5*w_p2))/2
                    y_p_mid = (y_p + (0.5*h_p) + y_p2 + (0.5*h_p2))/2
                    self.colour_frame = cv2.rectangle(self.colour_frame, (min(x_p, x_p2), min(y_p, y_p2)), (max((x_p+w_p), (x_p2+w_p2)), max((y_p+h_p), (y_p2+h_p2))), (200, 20, 0), 2)
                    self.get_logger().info('pinks are level')
                else:
                    x_p_mid = 0
                    y_p_mid = 0
            else:
                x_p_mid = 0
                y_p_mid = 0

            self.get_logger().info('Seeing pink')
        else:
            x_p = y_p = w_p = h_p = sec_lrg_contour = x_p2 = y_p2 = w_p2 = h_p2 = x_p_mid = y_p_mid = 0

        if x_g_mid != 0 and x_p_mid != 0:
            x_mid_diff = abs(x_g_mid - x_p_mid)
            self.centre_of_user = (x_g_mid + x_p_mid)/2
        else:
            x_mid_diff = 10    
        mask = mask_green + mask_pink
        self.mask = mask

        if x_mid_diff < 5 and y_g_mid < y_p_mid and y_g_mid != 0: # y values are zero at top and max at bottom. therefore if green is above pink its y value is LESS
            self.user_in_frame = True
            self.get_logger().info('User seen.')
            stripe_width_top = (max((x_g+w_g), (x_g2+w_g2))) - min(x_g, x_g2)
            stripe_width_bot = (max((x_p+w_p), (x_p2+w_p2))) - min(x_p, x_p2)
            self.stripe_width = round((stripe_width_top + stripe_width_bot)/2)
            self.get_logger().info(f'Stripe width = {self.stripe_width} pixels')
            GPIO.output(12,GPIO.HIGH)
        else:
            self.user_in_frame = False
            GPIO.output(12,GPIO.LOW)


    def callback_movement(self): # inputs are stripe width, obstacle and user seen flags, user centre error
        correct_stripe_width = 30
        correct_user_centre = 320 # camera width is 640. middle of camera = 640/2

        if self.obstacle_detected == True or self.user_in_frame == False:
            # set all movement to zero, maybe the setpoint?
            self.get_logger().info('No movement')
            return
        
        # PID loops, package not installed
        #pid_stripe = PID(1, 0.1, 0.05, setpoint=correct_stripe_width)
        #pid_user_error = PID(1, 0.1, 0.05, setpoint=correct_user_centre)

        # normalising to +-1


        
        

    def callback_cam_info(self, camera_info):
        self.K = np.array(camera_info.k).reshape([3,3])

              

# Main function
def main(args=None):
    rclpy.init(args=args)

    db = DetectBeacon()

    rclpy.spin(db)

if __name__ == '__main__':
    main()





