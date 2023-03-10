#!/usr/bin/env python3
import os
from pathlib import Path
import rospy
import cv2
import numpy as np

from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Pose, Point, TransformStamped, Vector3, Transform
from duckietown_msgs.msg import LEDPattern
from duckietown_msgs.srv import ChangePattern
import yaml
from dt_apriltags import Detector

from tf2_ros import TransformBroadcaster, Buffer, TransformListener

from tf import transformations as tr

class AugmentedRealityNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(AugmentedRealityNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.veh = rospy.get_param("~veh")
       
        #self.calibration_file = f'/data/config/calibrations/camera_intrinsic/{self.veh}.yaml'
        self.calibration_file = f'/data/config/calibrations/camera_intrinsic/default.yaml'
 
        self.calibration = self.readYamlFile(self.calibration_file)

        self.img_width = self.calibration['image_width']
        self.img_height = self.calibration['image_height']
        self.cam_matrix = np.array(self.calibration['camera_matrix']['data']).reshape((self.calibration['camera_matrix']['rows'], self.calibration['camera_matrix']['cols']))
        self.distort_coeff = np.array(self.calibration['distortion_coefficients']['data']).reshape((self.calibration['distortion_coefficients']['rows'], self.calibration['distortion_coefficients']['cols']))

        self.new_cam_matrix, self.roi = cv2.getOptimalNewCameraMatrix(self.cam_matrix, self.distort_coeff, (self.img_width, self.img_height), 1, (self.img_width, self.img_height))

        # setup april tag detector
        self.detector = Detector(searchpath=['apriltags'],
                       families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)


        # setup publisher
        self.undistorted = None
    
        # setup subscriber
        self.sub_img = rospy.Subscriber(f'/{self.veh}/camera_node/image/compressed', CompressedImage, self.get_img, queue_size = 1)

        # construct publisher
        self.pub_stop = rospy.Publisher(f'/{self.veh}/stopper', String, queue_size=1)

        # -- Proxy -- 
        led_service = f'/{self.veh}/led_controller_node/led_pattern'
        rospy.wait_for_service(led_service)
        self.led_pattern = rospy.ServiceProxy(led_service, ChangePattern)

        self._tf_broadcaster = TransformBroadcaster()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer)


    def get_img(self, msg):
        img = np.frombuffer(msg.data, np.uint8)
        img2 = cv2.imdecode(img, 0)     

        # https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
        undistorted = cv2.undistort(img2, self.cam_matrix, self.distort_coeff, None, self.new_cam_matrix)
        x, y, w, h = self.roi
        self.undistorted = undistorted[y:y+h, x:x+w]


    def detect_april(self):
        # https://pyimagesearch.com/2020/11/02/apriltag-with-python/
        # https://github.com/duckietown/lib-dt-apriltags/blob/master/test/test.py
        # april tag detection
        
        results = self.detector.detect(self.undistorted, estimate_tag_pose=True, camera_params=(self.cam_matrix[0,0], self.cam_matrix[1,1], self.cam_matrix[0,2], self.cam_matrix[1,2]), tag_size=0.065)

        # for r in results:
        # extract the bounding box (x, y)-coordinates for the AprilTag
        # and convert each of the (x, y)-coordinate pairs to integers
        try:
            r = results[0]

            t = tr.translation_from_matrix(tr.translation_matrix(np.array(r.pose_t).reshape(3)))
            d = np.linalg.norm(t)
            
            if (d < 0.5):
                print("STOP UR SHENANIGANS SENOR")
                self.stopper()

            if (r.tag_id == 93 or r.tag_id == 94 or r.tag_id == 200 or r.tag_id == 201):
                self.log('UOFA')
                self.change_led_lights("green")
            elif (r.tag_id == 62 or r.tag_id == 153 or r.tag_id == 133 or r.tag_id == 56):
                self.log('INTERSECTION')
                self.change_led_lights("blue")
            elif (r.tag_id == 162 or r.tag_id == 169):
                self.log('STOP')
                self.change_led_lights("red")


        except Exception as e:
            print(e)
            self.change_led_lights("white")

    def stopper(self):
        stop = String(data="stop")
        print(stop)
        self.pub_stop.publish(stop)

    def run(self):
        rate = rospy.Rate(3)
        self.change_led_lights("white")


        while not rospy.is_shutdown():
            
            if self.undistorted is not None:
                
                self.detect_april()

                rate.sleep()
            else:
                self.change_led_lights("white")

    def change_led_lights(self, color: str):
        '''
        Sends msg to service server
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
        msg.data = color
        self.led_pattern(msg)
            

    def readYamlFile(self,fname):
        """
        Reads the YAML file in the path specified by 'fname'.
        E.G. :
            the calibration file is located in : `/data/config/calibrations/filename/DUCKIEBOT_NAME.yaml`
        """
        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
                return yaml_dict
            except yaml.YAMLError as exc:
                self.log("YAML syntax error. File: %s fname. Exc: %s"
                        %(fname, exc), type='fatal')
                rospy.signal_shutdown()
                return


if __name__ == '__main__':
    # create the node
    node = AugmentedRealityNode(node_name='augmented_reality_node')
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass