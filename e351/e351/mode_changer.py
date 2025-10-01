import rclpy
from rclpy.node import Node
import cv2

class DetectBeacon(Node):
    def __init__(self):
        # Initialise subs, pubs, service calls, path object
        super().__init__('detect_beacon')

        self.park_mode = True


    def callback_mode(self):
        resp = cv2.waitKey(80)
        button_latch = False

        if self.park_mode == True:
            self.get_logger().info('Current mode: Park Mode')
        else:
            self.get_logger().info('Current mode: Follower Mode')

        if resp == ord('m') and button_latch == False: # change mode button
            if self.park_mode == True:
                self.park_mode = False
                self.get_logger().info('Changing from park mode to follower mode...')
            else:
                self.park_mode = True
                self.get_logger().info('Changing from follower mode to park mode...')
            button_latch = True
        elif resp != ord('m') and button_latch == True:
            button_latch = False

# Main function
def main(args=None):
    rclpy.init(args=args)

    db = DetectBeacon()

    rclpy.spin(db)

if __name__ == '__main__':
    main()
