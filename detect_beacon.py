import rclpy
from rclpy.node import Node
import imutils
import numpy as np
import cv2
import RPi.GPIO as GPIO
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
from simple_pid import PID
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy


class DetectBeacon(Node):
    def __init__(self):
        super().__init__('detect_beacon')
        self.get_logger().info('Starting Detect Beacon Node')
        qos_policy = QoSProfile(durability=QoSDurabilityPolicy.VOLATILE, reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=5)

        # Initialising subscribers
        self.subscriber_colour = self.create_subscription(Image, '/image_raw', self.callback_imgpro, qos_policy)
        #self.subscriber_depth = self.create_subscription(Image, '/intel_realsense_r200_depth/depth/image_raw', self.callback_depth, qos_policy)
        self.subscriber_camera_info = self.create_subscription(CameraInfo, '/camera_info', self.callback_cam_info, qos_policy)

        # Initialising flags        
        self.bridge = CvBridge()
        self.colour_frame = None
        self.depth_frame = None
        self.K = None
        self.obstacle_detected = False
        self.ob_latch = False
        self.user_in_frame = False
        self.park_mode = True
        self.button_latch = False
        self.speed = 0
        self.steer = 0
        self.lidar_distance = 1

        print('Now in park mode.')

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(16,GPIO.OUT) # park mode LED
        GPIO.setup(4,GPIO.OUT) # follower mode LED
        GPIO.setup(12,GPIO.OUT) # user seen LED
        GPIO.setup(22,GPIO.IN) # mode changer button
        GPIO.setup(17,GPIO.OUT, initial=GPIO.HIGH) # AIN1
        GPIO.setup(18,GPIO.OUT, initial=GPIO.LOW) # PWMA
        GPIO.setup(23,GPIO.OUT, initial=GPIO.HIGH) # BIN1
        GPIO.setup(24,GPIO.OUT, initial=GPIO.LOW) # PWMB
        self.left_pwm = GPIO.PWM(18, 1000)
        self.right_pwm = GPIO.PWM(24, 1000)
        self.left_pwm.start(0.0)
        self.right_pwm.start(0.0)

        # Timers to invoke callbacks
        self.timer_mode = self.create_timer(0.1, self.callback_mode)
        self.timer_set_move = self.create_timer(0.1, self.callback_set_movement)
        self.timer_do_move = self.create_timer(0.1, self.callback_do_movement)

    
    def callback_mode(self): # To change modes
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
                print('Now in follower mode.')
            else:
                self.park_mode = True
                print('Now in park mode.')
            self.button_latch = True
        elif button_on == 0 and self.button_latch == True:
            self.button_latch = False

    def callback_depth(self):
        if self.park_mode == True:
            return
        
        #self.lidar_distance = GPIO.input(X) # may need to scale
        
        if self.lidar_distance < 0.3:
            self.obstacle_detected = True
            if self.ob_latch == False: # Latching so it only prints message once
                print('Obstacle detected! Stopping movement & image processing.')
            self.ob_latch = True
        else:
            self.obstacle_detected = False
            if self.ob_latch == True:
                print('Obstacle cleared! Continuing movement.')
            self.ob_latch = False
    
    def callback_imgpro(self, colour_image): # Image processing
        if self.park_mode == True or self.obstacle_detected == True: # if in park or obstacle detected, don't bother going through image processing
            return
        #self.get_logger().info('Now starting image processing.')
        self.colour_frame = self.bridge.imgmsg_to_cv2(colour_image, "bgr8")
        blurred = cv2.GaussianBlur(self.colour_frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        green_lower = (48, 150, 80)
        green_upper = (58, 255, 255)
        pink_lower = (160, 150, 80)
        pink_upper = (170, 255, 255)

        # Green detection
        mask_green = cv2.inRange(hsv, green_lower, green_upper)
        mask_green = cv2.erode(mask_green, None, iterations=2)
        mask_green = cv2.dilate(mask_green, None, iterations=2)
        contours = cv2.findContours(mask_green.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 
        contours = imutils.grab_contours(contours)
        if len(contours) != 0:
            largest_contour = max(contours, key=cv2.contourArea)
            x_g, y_g, w_g, h_g = cv2.boundingRect(largest_contour)
            #self.get_logger().info(f'green len(contours) = {len(contours)}')
            #self.get_logger().info(f'x_g = {x_g}, y_g = {y_g}, w_g = {w_g}, h_g = {h_g}')

            if len(contours) >= 2: #if it sees 2 or more patches of this green, which it should if it sees the stripe
                i=0
                while i <= len(contours):
                    x, y, w, h = cv2.boundingRect(contours[i])
                    if x == x_g and y == y_g and w == w_g and h == h_g: #got _g values before.
                        cont = list(contours)
                        cont.pop(i)
                        contours = tuple(cont)
                        break
                    i+=1
                sec_lrg_contour = max(contours, key=cv2.contourArea)
                x_g2, y_g2, w_g2, h_g2 = cv2.boundingRect(sec_lrg_contour)
                #self.get_logger().info(f'x_g2 = {x_g2}, y_g2 = {y_g2}, w_g2 = {w_g2}, h_g2 = {h_g2}')
                if abs(y_g - y_g2) < 10: #if largest and second largest contours are (almost) in line, its the green we want
                    x_g_mid = (x_g + (0.5*w_g) + x_g2 + (0.5*w_g2))/2 #avg of mid of both contours
                    y_g_mid = (y_g + (0.5*h_g) + y_g2 + (0.5*h_g2))/2
                    self.colour_frame = cv2.rectangle(self.colour_frame, (min(x_g, x_g2), min(y_g, y_g2)), (max((x_g+w_g), (x_g2+w_g2)), max((y_g+h_g), (y_g2+h_g2))), (200, 20, 0), 2) 
                    #self.get_logger().info('greens are level')
                else:
                    x_g_mid = 0
                    y_g_mid = 0
            else:
                x_g_mid = 0
                y_g_mid = 0

        else:
            x_g = y_g = w_g = h_g = sec_lrg_contour = x_g2 = y_g2 = w_g2 = h_g2 = x_g_mid = y_g_mid = 0

        # Pink detection
        mask_pink = cv2.inRange(hsv, pink_lower, pink_upper)
        mask_pink = cv2.erode(mask_pink, None, iterations=2)
        mask_pink = cv2.dilate(mask_pink, None, iterations=2)
        contours = cv2.findContours(mask_pink.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 
        contours = imutils.grab_contours(contours)
        if len(contours) != 0:
            largest_contour = max(contours, key=cv2.contourArea)
            x_p, y_p, w_p, h_p = cv2.boundingRect(largest_contour)
            #self.get_logger().info(f'pink len(contours) = {len(contours)}')
            #self.get_logger().info(f'x_p = {x_p}, y_p = {y_p}, w_p = {w_p}, h_p = {h_p}')

            if len(contours) >=2:
                i=0
                while i <= len(contours):
                    x, y, w, h = cv2.boundingRect(contours[i])
                    if x == x_p and y == y_p and w == w_p and h == h_p: #got _g values before.
                        cont = list(contours)
                        cont.pop(i)
                        contours = tuple(cont)
                        break
                    i+=1
                sec_lrg_contour = max(contours, key=cv2.contourArea)
                x_p2, y_p2, w_p2, h_p2 = cv2.boundingRect(sec_lrg_contour)
                #self.get_logger().info(f'x_p2 = {x_p2}, y_p2 = {y_p2}, w_p2 = {w_p2}, h_p2 = {h_p2}')
                if abs(y_p - y_p2) < 10:
                    #self.get_logger().info('pinks are level')
                    x_p_mid = (x_p + (0.5*w_p) + x_p2 + (0.5*w_p2))/2
                    y_p_mid = (y_p + (0.5*h_p) + y_p2 + (0.5*h_p2))/2
                    self.colour_frame = cv2.rectangle(self.colour_frame, (min(x_p, x_p2), min(y_p, y_p2)), (max((x_p+w_p), (x_p2+w_p2)), max((y_p+h_p), (y_p2+h_p2))), (200, 20, 0), 2)
                else:
                    x_p_mid = 0
                    y_p_mid = 0
            else:
                x_p_mid = 0
                y_p_mid = 0

        else:
            x_p = y_p = w_p = h_p = sec_lrg_contour = x_p2 = y_p2 = w_p2 = h_p2 = x_p_mid = y_p_mid = 0

        # Determining centre of user
        if x_g_mid != 0 and x_p_mid != 0:
            x_mid_diff = abs(x_g_mid - x_p_mid)
            self.centre_of_user = (x_g_mid + x_p_mid)/2
        else:
            x_mid_diff = 10
            self.centre_of_user = 0

        # Is the user seen?
        if x_mid_diff < 5 and y_g_mid < y_p_mid and y_g_mid != 0: # y values are zero at top and max at bottom. therefore if green is above pink its y value is LESS
            self.user_in_frame = True
            stripe_width_top = (max((x_g+w_g), (x_g2+w_g2))) - min(x_g, x_g2)
            stripe_width_bot = (max((x_p+w_p), (x_p2+w_p2))) - min(x_p, x_p2)
            self.stripe_width = round((stripe_width_top + stripe_width_bot)/2)
            print(f'User seen. Stripe width = {self.stripe_width} pixels')
            GPIO.output(12,GPIO.HIGH)
        else:
            self.user_in_frame = False
            GPIO.output(12,GPIO.LOW)


    def callback_set_movement(self): # inputs are stripe width, user centre error, obstacle and user seen and park mode flags. outputs are 2 normalised values
        # Set PID setpoints
        if self.obstacle_detected == True or self.user_in_frame == False or self.park_mode == True:
            # Set all movement to zero
            #self.get_logger().info('No movement')
            self.norm_stripe = 0
            self.norm_user = 0
            self.stop_cmd = True
            return
        else:
            self.stop_cmd = False

        # Normalising stripe width (should be [90,190]) to [0,1]. 
        if self.stripe_width > 190 or self.stripe_width == 0: # If wider than 190 pixels (closer than min following distance) or can't see (should be able to but just in case), set point to zero
            self.norm_stripe = 0
        #elif self.stripe_width < 90:
            #self.norm_stripe = 1
        else:
            self.norm_stripe = 1*(self.stripe_width - 190)/100

        # Normalising centre of user (should be [0,640]) to [-1,1]. camera width = 640, therefore centre = 640/2 = 320
        self.norm_user = (self.centre_of_user - 320)/320


    def callback_do_movement(self): # inputs are normalised stripe and user values
        # Statement to not execute code if not required (i.e. set point is zero and speed is zero)
        if self.norm_stripe == 0 and self.speed == 0:
            return
  
        # PID loops, more aggressive PID loop for when trailer needs to stop
        pid_stop = PID(1, 0.1, 0.05, setpoint=0)
        pid_stripe = PID(0.5, 0.1, 0.05, setpoint=0) # 0 is vest at 190 pixels wide which is ideal following distance
        pid_steering = PID(0.7, 0.1, 0.05, setpoint=0) # 0 is user at centre which is ideal

        if self.stop_cmd == True:
            self.speed = round(pid_stop(self.norm_stripe), 3)
        else:
            self.speed = round(pid_stripe(self.norm_stripe), 3)
        self.steer = round(pid_steering(self.norm_user), 3)

        if self.speed > 1:
            self.speed = 1
        elif self.speed < 0:
            self.speed = 0

        #self.get_logger().info(f'self.speed = {self.speed}')
        #self.get_logger().info(f'self.steer = {self.steer}')
        
        # Steering multipliers with +-0.05 deadband
        if self.steer > -1 and self.steer < -0.05:
            left_mult = 1 + self.steer
            right_mult = 1
            #self.get_logger().info(f'Veering left by {left_mult}')
        elif self.steer > 0.05 and self.steer < 1:
            left_mult = 1
            right_mult = 1 - self.steer
            #self.get_logger().info(f'Veering right by {right_mult}')
        else:
            left_mult = 1
            right_mult = 1
            #self.get_logger().info('Going straight.')

        # PWM signals to GPIO
        left_pwm_signal = round(self.speed * left_mult * 100) # must be whole number bewteen 0 and 100
        right_pwm_signal = round(self.speed * right_mult * 100)
        
        self.left_pwm.ChangeDutyCycle(left_pwm_signal)
        self.right_pwm.ChangeDutyCycle(right_pwm_signal)

        print(f'Left PWM Signal: {left_pwm_signal}  Right PWM Signal: {right_pwm_signal}')

        
    def callback_cam_info(self, camera_info):
        self.K = np.array(camera_info.k).reshape([3,3])

              

# Main function
def main(args=None):
    rclpy.init(args=args)

    db = DetectBeacon()

    rclpy.spin(db)

if __name__ == '__main__':
    main()



