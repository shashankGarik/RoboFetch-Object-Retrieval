from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

from visualization_msgs.msg import MarkerArray, Marker

from geometry_msgs.msg import PointStamped, Transform, TransformStamped

from sensor_msgs.msg import PointCloud2

from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

from joint_controller import JointController, Joints
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Int8



import math
import time
import threading
import sys

import tf2_ros
import argparse as ap        
import numpy as np
import threading
import ros_numpy as rn

import hello_helpers.hello_misc as hm
import stretch_funmap.navigate as nv


class PersonDetectionNode():#hm.HelloNode):

    def __init__(self):
        # hm.HelloNode.__init__(self)
        self.rate = 10.0
        self.joint_states = None
        self.joint_states_lock = threading.Lock()
        self.move_base = nv.MoveBase(self)
        self.letter_height_m = 0.2
        self.wrist_position = None
        self.lift_position = None
        self.manipulation_view = None

        self.joint_controller = JointController()
        
        marker = Marker()
        self.mouth_marker_type = marker.CUBE
        self.mouth_point = None

        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)


        self.tilt_angle = 0.0#-0.5

        # self.prev_pan_index = 0
        
        self.move_lock = threading.Lock()

        self.curr_wrist_extension = 0.0
        self.curr_lift = 0.0
        self.curr_yaw = 0.0
        self.curr_gripper = 0.0
        self.curr_head_pan = 0.0
        self.curr_head_tilt = 0.0

        self.mouth_markers = []

        self.move_flag = 1

        self.trans_base = TransformStamped()
        self.trans_camera = TransformStamped()

        # self.joint_states_subscriber = rospy.Subscriber('joint_states', JointState, self.joint_state_callback)
        
        self.mouth_position_subscriber = rospy.Subscriber('/nearest_mouth/marker_array', MarkerArray, self.mouth_position_callback)

        # self.move_subscriber = rospy.Subscriber('/move_to_person', Int8, self.detect_mouth_callback, queue_size=10)


    def detect_mouth_callback(self, move_flag):
        if move_flag.data == 1:
            self.main_function()

    def mouth_position_callback(self, msg):
        self.mouth_markers = msg.markers
    
    def main_function(self):
        # for marker in self.mouth_markers:
        # if marker.type == self.mouth_marker_type:
        if self.mouth_markers[0].type == self.mouth_marker_type:
            marker = self.mouth_markers[0]
        
            mouth_frame_id = marker.header.frame_id
            lookup_time = rospy.Time(0) # return most recent transform
            timeout_ros = rospy.Duration(0.1)
            self.trans_base = self.tf2_buffer.lookup_transform('base_link', mouth_frame_id, lookup_time, timeout_ros)

            x_t = self.trans_base.transform.translation.x
            y_t = self.trans_base.transform.translation.y
            z_t = self.trans_base.transform.translation.z

            print('x dist:', x_t)
            print('y dist:', y_t)
            print('z dist:', z_t)
            print()
            rospy.sleep(rospy.Duration(5))

if __name__ == '__main__':
    rospy.init_node('person_detection')
    person_detection = PersonDetectionNode()

    person_detection.main_function() #COMMENT OUT
    rospy.spin()
