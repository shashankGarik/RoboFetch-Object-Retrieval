import rospy
import math
import time
import actionlib
import os
import csv
from datetime import datetime
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectoryPoint
from joint_controller import JointController, Joints
from sensor_msgs.msg import JointState
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_geometry_msgs
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, Transform, TransformStamped
from visualization_msgs.msg import MarkerArray
from rail_stretch_navigation.srv import GraspAruco
from manipulate_arm import Manipulate_Arm
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import ros_numpy as rn
import numpy as np
import cv2

from mask_rcnn import MaskRCNN



class DepthCamera:
    def __init__(self):

        self.mrcnn = MaskRCNN()

        # self.image = np.zeros((1920,1080))
        self.bridge = CvBridge()

        # self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback, queue_size=1)
        # pass

        # self.depth_image = np.empty((240, 424))
        # self.rgb_image = np.empty((240, 424))
        self.depth_image = Image()
        self.rgb_image = Image()

        # self.img = UInt8MultiArray()
        
    def depth_callback(self, img):

        # image_np = self.bridge.imgmsg_to_cv2(img) #For Image type
        # print(type(image_np))

        # print(image_np.shape, image_np.size)  # - (240, 424) 101760
        # print(image_np.dtype) # - uint16

        image_np = self.bridge.compressed_imgmsg_to_cv2(img) #For CompressedImage type


        #Another way to convert bytes to image array
        # np_arr = np.fromstring(img.data, np.uint8)
        # # print(np_arr.shape, np_arr.size)
        # image_np = cv2.imdecode(np_arr, cv2.COLOR_BGR2GRAY)
        
        

        # print(image_np.shape, image_np.size)  # - (240, 424), 101760
        # print(image_np.dtype) # - uint8

        rotated_img = cv2.rotate(image_np, cv2.ROTATE_90_CLOCKWISE)
        # print(type(rotated_img))

        # cv2.imshow('depth_frame', rotated_img)
        # cv2.waitKey(1)

        self.depth_image = rotated_img



    def image_callback(self, img):

        # image_np = self.bridge.imgmsg_to_cv2(img) #For Image type

        image_np = self.bridge.compressed_imgmsg_to_cv2(img) #For CompressedImage type

        image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)

        rotated_img = cv2.rotate(image_np, cv2.ROTATE_90_CLOCKWISE)

        # cv2.imshow('rgb_frame', rotated_img)
        # cv2.waitKey(1)

        # print(type(self.depth_image))


        self.rgb_image = rotated_img
        
        # rospy.sleep(rospy.Duration(2))
        # self.show_image(self.depth_image)


        # Get object mask
        boxes, classes, contours, centers = self.mrcnn.detect_objects_mask(self.rgb_image)

        # Draw object mask
        self.rgb_image = self.mrcnn.draw_object_mask(self.rgb_image)

        # Show depth info of the objects
        self.mrcnn.draw_object_info(self.rgb_image, self.depth_image)

        self.show_image(self.rgb_image)



    def show_image(self, rgb_img):
        # print(rgb_img)
        
        # cv2.imshow("depth frame", depth_img) # - (720, 1280) 921600
        cv2.imshow("Bgr frame", rgb_img)
        cv2.waitKey(1)



            
    def main(self):
        rospy.init_node('DepthCamera')
        self.node_name = rospy.get_name()
        rospy.loginfo("{0} started".format(self.node_name))

        # self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.image_sub = rospy.Subscriber('/camera/color/image_raw/compressed', CompressedImage, self.image_callback)


        # rospy.sleep(rospy.Duration(2))
        # self.depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw/compressed', CompressedImage, self.depth_callback)



if __name__ == '__main__':
    node = DepthCamera()
    node.main()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('interrupt received, so shutting down')

