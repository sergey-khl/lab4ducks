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

ROAD_MASK = [(20, 60, 0), (50, 255, 255)]
STOP_MASK = [(0, 60, 120), (15, 255, 255)]
DEBUG = False
ENGLISH = False

class LaneFollowNode(DTROS):

    def __init__(self, node_name):
        super(LaneFollowNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
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
        # self.sub_stop = rospy.Subscriber("/" + self.veh + "/stopper",
        #                             String,
        #                             self.cb_stop,
        #                             queue_size=1)
        self.sub_dist = rospy.Subscriber("/" + self.veh + "/duckiebot_distance_node/distance", Point, self.cb_dist, queue_size=1)
        self.vel_pub = rospy.Publisher("/" + self.veh + "/car_cmd_switch_node/cmd",
                                       Twist2DStamped,
                                       queue_size=1)

        self.jpeg = TurboJPEG()

        self.delay = 0
        self.stopping = False
        self.turning = False
        self.update = False

        self.following = -1

        self.loginfo("Initialized")

        # PID Variables
        self.proportional = None
        if ENGLISH:
            self.offset = -220
        else:
            self.offset = 220
        self.velocity = 0.4
        self.twist = Twist2DStamped(v=self.velocity, omega=0)

        self.P = 0.049
        self.D = -0.004
        self.last_error = 0
        self.last_time = rospy.get_time()

        # -- Proxy -- 
        led_service = f'/{self.veh}/led_controller_node/led_pattern'
        rospy.wait_for_service(led_service)
        self.led_pattern = rospy.ServiceProxy(led_service, ChangePattern)

        # Shutdown hook
        rospy.on_shutdown(self.hook)

    def callback(self, msg):
        # if (self.following):
        #     return
        img = self.jpeg.decode(msg.data)
        crop = img[300:-1, :, :]
        crop_width = crop.shape[1]
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        maskY = cv2.inRange(hsv, ROAD_MASK[0], ROAD_MASK[1])
        cropY = cv2.bitwise_and(crop, crop, mask=maskY)
        contoursY, hierarchy = cv2.findContours(maskY,
                                               cv2.RETR_EXTERNAL,
                                               cv2.CHAIN_APPROX_NONE)
        maskR = cv2.inRange(hsv, STOP_MASK[0], STOP_MASK[1])
        cropR = cv2.bitwise_and(crop, crop, mask=maskR)
        contoursR, hierarchy = cv2.findContours(maskR,
                                               cv2.RETR_EXTERNAL,
                                               cv2.CHAIN_APPROX_NONE)


        # Search for lane in front
        max_area = 20
        max_idx = -1

        for i in range(len(contoursY)):
            area = cv2.contourArea(contoursY[i])
            if area > max_area:
                max_idx = i
                max_area = area

        if max_idx != -1:
            M = cv2.moments(contoursY[max_idx])

            try:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                
                self.proportional = cx - int(crop_width / 2) + self.offset

                if (self.update):
                    self.turning = True
                    if (self.proportional > 200):
                        self.change_led_lights("2")
                        print('going right')
                        self.turn(-8, 1.2, 1)
                    else:
                        self.change_led_lights("0")
                        print('going straight')
                        self.turn(0, 0.7, 1)
                    self.change_led_lights("0")
                    self.update = False
                    self.turning = False
                if DEBUG:
                    cv2.drawContours(cropY, contoursY, max_idx, (0, 255, 0), 3)
                    cv2.circle(cropY, (cx, cy), 7, (0, 0, 255), -1)
            except:
                pass
        else:
            self.proportional = None

        # Search for lane in front
        max_area = 20
        max_idx = -1
        for i in range(len(contoursR)):
            area = cv2.contourArea(contoursR[i])
            if area > max_area:
                max_idx = i
                max_area = area
        
        if max_idx != -1:
            M = cv2.moments(contoursR[max_idx])
            try:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                # stop at red line then turn

                if (self.delay <= 0 and cy > 140):
                    self.turning = True
                    self.change_led_lights(str(self.following))
                    rate = rospy.Rate(1)
                    self.twist.v = 0
                    self.twist.omega = 0
                    for i in range(8):
                        self.vel_pub.publish(self.twist)
                    rate.sleep()
                    
                    if (self.following == 0):
                        print('going straight')
                        self.turn(0, 0.3, 2)
                    elif (self.following == 1):
                        print('going left')
                        self.turn(3, 0.5, 1.8)
                    elif (self.following == 2):
                        print('going right')
                        self.turn(-3, 0.5, 1.8)
                    else:
                        print('default')
                        self.turn(0, 3, 2)
                        self.update = True
                    
                    self.turning = False

                if DEBUG:
                    cv2.drawContours(cropR, contoursR, max_idx, (0, 255, 0), 3)
                    cv2.circle(cropR, (cx, cy), 7, (0, 0, 255), -1)
            except:
                pass

        if DEBUG:
            rect_img_msg = CompressedImage(format="jpeg", data=self.jpeg.encode(crop))
            self.pub.publish(rect_img_msg)
    
    def turn(self, omega, hz, delay):
        rate = rospy.Rate(hz)
        self.twist.v = self.velocity
        self.twist.omega = omega
        for i in range(8):
            self.vel_pub.publish(self.twist)
        rate.sleep()
        self.delay = delay


    def cb_dist(self, msg):
        print(msg)
        self.stopping = False
        if (msg.x == msg.y and msg.y == msg.z and msg.z == 0):
            # do ya own thang
            self.following = -1
        elif (msg.z < 0.5):
            self.stopping = True
        else:
            
            if (msg.x < -0.15):
                # go left
                self.following = 1
            elif (msg.x > 0.15):
                # go right
                self.following = 2
            else:
                # go straight
                self.following = 0
                


    def drive(self):
        self.delay -= (rospy.get_time() - self.last_time)

        if not self.turning:
            if self.stopping:

                self.twist.v = 0
                self.twist.omega = 0
                
            elif self.proportional is None:

                self.twist.v = self.velocity
                self.twist.omega = 0
            else:

                # P Term
                P = -self.proportional * self.P
                
                # D Term
                d_error = (self.proportional - self.last_error) / (rospy.get_time() - self.last_time)
                self.last_error = self.proportional
                
                D = d_error * self.D

                self.twist.v = self.velocity
                self.twist.omega = P + D
                if DEBUG:
                    self.loginfo(self.proportional, P, D, self.twist.omega, self.twist.v)

            self.vel_pub.publish(self.twist)
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