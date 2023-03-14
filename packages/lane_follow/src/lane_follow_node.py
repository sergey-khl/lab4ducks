#!/usr/bin/env python3

import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Float32, String
from turbojpeg import TurboJPEG
import cv2
import numpy as np
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped
from geometry_msgs.msg import Point
from duckietown_msgs.msg import LEDPattern
from duckietown_msgs.srv import ChangePattern

# Define the HSV color range for road and stop mask
ROAD_MASK = [(20, 60, 0), (50, 255, 255)]
STOP_MASK = [(0, 120, 120), (15, 255, 255)]

# Set debugging mode (change to True to enable)
DEBUG = False

# Determines what side of the road the robot drives on (change to True to dirve on left)
ENGLISH = False

class LaneFollowNode(DTROS):
    def __init__(self, node_name):
        super(LaneFollowNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        # Save node name and vehicle name
        self.node_name = node_name
        self.veh = rospy.get_param("~veh")

        # Publishers & Subscribers
        self.pub = rospy.Publisher("/" + self.veh + "/output/image/mask/compressed",
                                   CompressedImage,
                                   queue_size=1)
        self.sub = rospy.Subscriber("/" + self.veh + "/camera_node/image/compressed",
                                    CompressedImage,
                                    self.callback,
                                    queue_size=1,
                                    buff_size="20MB")
        

        # Initialize distance subscriber and velocity publisher
        self.sub_dist = rospy.Subscriber("/" + self.veh + "/duckiebot_distance_node/distance", Point, self.cb_dist, queue_size=1)
        self.vel_pub = rospy.Publisher("/" + self.veh + "/car_cmd_switch_node/cmd",
                                       Twist2DStamped,
                                       queue_size=1)
        
        # Initialize TurboJPEG decoder
        self.jpeg = TurboJPEG()

        # Initialize driving behaviour variables
        self.delay = 0
        self.stopping = False
        self.turning = False
        self.update = False
        self.following = -1

        self.velocity = 0.4
        self.twist = Twist2DStamped(v=self.velocity, omega=0)

        self.offset = 220

        if ENGLISH:
            self.offset *= -1

        # Initialize lane following PID Variables
        self.proportional_lane_following = None
        self.P_gain_lane_following = 0.049
        self.D_gain_lane_following = -0.004
        self.last_error_lane_following = 0

        # Initialize collision PID Variables
        self.proportional_collision = None
        self.P_gain_collision = 0.049
        self.D_gain_collision = 0
        self.last_error_lane_following = 0

        self.last_time = rospy.get_time()

        # Initialize LED pattern change service
        led_service = f'/{self.veh}/led_controller_node/led_pattern'
        rospy.wait_for_service(led_service)
        self.led_pattern = rospy.ServiceProxy(led_service, ChangePattern)

        # Initialize shutdown hook
        rospy.on_shutdown(self.hook)


    def callback(self, msg):
        # decode the received message to an image
        img = self.jpeg.decode(msg.data)
        crop = img[300:-1, :, :]
        crop_width = crop.shape[1]

        # convert the cropped image to HSV color space
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)

        # create a binary mask for yellow color
        maskY = cv2.inRange(hsv, ROAD_MASK[0], ROAD_MASK[1])
        cropY = cv2.bitwise_and(crop, crop, mask=maskY)

         # find contours in the binary mask
        contoursY, hierarchy = cv2.findContours(maskY,
                                               cv2.RETR_EXTERNAL,
                                               cv2.CHAIN_APPROX_NONE)
        
        # create a binary mask for red color
        maskR = cv2.inRange(hsv, STOP_MASK[0], STOP_MASK[1])
        cropR = cv2.bitwise_and(crop, crop, mask=maskR)

        # find contours in the binary mask
        contoursR, hierarchy = cv2.findContours(maskR,
                                               cv2.RETR_EXTERNAL,
                                               cv2.CHAIN_APPROX_NONE)

        # Search for lane using yellow color
        max_area = 20
        max_idx = -1

        for i in range(len(contoursY)):
            # find the contour with the largest area
            area = cv2.contourArea(contoursY[i])
            if area > max_area:
                max_idx = i
                max_area = area

        if max_idx != -1:
            # calculate the center of the lane using moments
            M = cv2.moments(contoursY[max_idx])
            try:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                # calculate the error between the center of the lane and the center of the image
                self.proportional_lane_following = cx - int(crop_width / 2) + self.offset

                # if the error is within a threshold, move straight, otherwise turn right or left
                if self.update and not self.stopping and not self.turning:
                    self.turning = True
                    if (self.proportional_lane_following > 200):
                        self.change_led_lights("2")
                        print('default right')
                        self.turn(-8, 1.7, 1.8)
                    else:
                        self.change_led_lights("0")
                        print('default straight')
                        self.turn(0, 0.7, 2)

                    self.change_led_lights("0")
                    self.update = False
                    self.turning = False

                if DEBUG:
                    cv2.drawContours(cropY, contoursY, max_idx, (0, 255, 0), 3)
                    cv2.circle(cropY, (cx, cy), 7, (0, 0, 255), -1)
            except:
                pass
        else:
            self.proportional_lane_following = None

        # Search for stop line using red color
        max_area = 20
        max_idx = -1

        for i in range(len(contoursR)):
            # find the contour with the largest area
            area = cv2.contourArea(contoursR[i])
            if area > max_area:
                max_idx = i
                max_area = area
        
        if max_idx != -1:
            M = cv2.moments(contoursR[max_idx])
            try:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                 # If the robot has reached the red line, it stops and turns
                 
                if self.delay <= 0 and cy > 140 and not self.stopping and not self.turning:
                    self.turning = True
                    self.delay = 2
                    print('stop, red line')
                    # Changes the LED lights according to the direction
                    # it's going to turn (straight, left, right, default)
                    self.change_led_lights(str(self.following))

                    # Publishes velocity messages to stop the robot
                    rate = rospy.Rate(1)
                    self.twist.v = 0
                    self.twist.omega = 0
                    for i in range(8):
                        self.vel_pub.publish(self.twist)
                    rate.sleep()
                    
                    # The robot turns depending on the direction it was following
                    if (self.following == 0):
                        print('going straight')
                        self.turn(0, 0.3, 2)
                    elif (self.following == 1):
                        print('going left')
                        self.turn(3, 0.5, 1.8)
                    elif (self.following == 2):
                        print('going right')
                        self.turn(-4, 0.5, 1.8)
                    else:
                        self.turn(0, 3, 2)
                        self.update = True

                    # Changes LED lights to default
                    self.change_led_lights("0")
                    self.turning = False

                # Draws the contour and the centroid on the image if in DEBUG mode
                if DEBUG:
                    cv2.drawContours(cropR, contoursR, max_idx, (0, 255, 0), 3)
                    cv2.circle(cropR, (cx, cy), 7, (0, 0, 255), -1)
        
            except:
                pass

        if DEBUG:
            rect_img_msg = CompressedImage(format="jpeg", data=self.jpeg.encode(crop))
            self.pub.publish(rect_img_msg)


    def turn(self, omega, hz, delay):
        # Set the rate at which to publish the twist messages
        rate = rospy.Rate(hz)
        # Set the delay before the next movement
        self.delay = delay

        # Set the linear velocity of the robot
        self.twist.v = self.velocity

        # Set the angular velocity of the robot
        self.twist.omega = omega

        # Publish the twist message multiple times to ensure the robot turns
        for i in range(8):
            self.vel_pub.publish(self.twist)

        # Sleep for the desired time to allow the robot to turn
        rate.sleep()
        

    def cb_dist(self, msg):
        
        # If the distance values received are all 0, set following to -1
            if msg.x == msg.y and msg.y == msg.z and msg.z == 0 and not self.stopping:
                # do ya own thang
                self.following = -1
            else:
                if (msg.x < -0.1):
                    # go left
                    self.following = 1
                elif (msg.x > 0.1):
                    # go right
                    self.following = 2
                else:
                    # go straight
                    self.following = 0

                # If the z-value is less than 0.5, set stopping to True
                if (msg.z < 0.5):
                    self.stopping = True
                else:
                    self.stopping = False
                

    def drive(self):
        # decrease delay by elapsed time
        self.delay -= (rospy.get_time() - self.last_time)
        if self.stopping:
            print('stopping behind bot')
            # stop robot
            self.twist.v = 0
            self.twist.omega = 0
            self.vel_pub.publish(self.twist)

        # check if the robot is turning or stopping
        elif not self.turning:
            try:
                # P Term
                P = -self.proportional_lane_following * self.P_gain_lane_following
                
                # D Term
                d_error = (self.proportional_lane_following - self.last_error_lane_following) / (rospy.get_time() - self.last_time)
                D = d_error * self.D_gain_lane_following

                self.last_error_lane_following = self.proportional_lane_following

                # set linear and angular velocity
                self.twist.v = self.velocity
                self.twist.omega = P + D

                if DEBUG:
                    self.loginfo(self.proportional_lane_following, P, D, self.twist.omega, self.twist.v)

            except:
                # robot is lost and doesn't know where to go
                print('idk where I am')
                self.twist.v = self.velocity
                self.twist.omega = 0


            # publish twist message
            self.vel_pub.publish(self.twist)
            
        # update last time
        self.last_time = rospy.get_time()


    def hook(self):
        print("SHUTTING DOWN")
        self.twist.v = 0
        self.twist.omega = 0
        self.vel_pub.publish(self.twist)

        for i in range(8):
            self.vel_pub.publish(self.twist)


    def change_led_lights(self, dir: str):
        '''
        Sends msg to service server
        ignore
        Colors:
            "off": [0,0,0],
            "white": [1,1,1],
            "green": [0,1,0],
            "red": [1,0,0],
            "blue": [0,0,1],
            "yellow": [1,0.8,0],
            "purple": [1,0,1],
            "cyan": [0,1,1],
            "pink": [1,0,0.5],
        '''
        msg = String()
        msg.data = dir
        self.led_pattern(msg)


if __name__ == "__main__":
    node = LaneFollowNode("lanefollow_node")
    rate = rospy.Rate(8)  # 8hz
    node.change_led_lights("0")
    while not rospy.is_shutdown():
        node.drive()
        rate.sleep()