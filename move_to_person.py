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
        self.x = 0
        self.y = 0
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
        self.pub = rospy.Publisher('/stretch/cmd_vel', Twist, queue_size=1)

        # self.move_subscriber = rospy.Subscriber('/move_to_person', Int8, self.detect_mouth_callback, queue_size=10)

        self.switch_base_to_navigation = rospy.ServiceProxy('/switch_to_navigation_mode', Trigger)
        self.switch_base_to_navigation()

        


    def detect_mouth_callback(self, move_flag):
        pass
        # if move_flag.data == 1:
        #     self.main_function()

    def mouth_position_callback(self, msg):
        self.mouth_markers = msg.markers

        for marker in self.mouth_markers:
            if marker.type == self.mouth_marker_type:
            # if self.mouth_markers[0].type == self.mouth_marker_type:
                # marker = self.mouth_markers
            
                self.mouth_frame_id = marker.header.frame_id
                # lookup_time = rospy.Time(0) # return most recent transform
                # timeout_ros = rospy.Duration(0.1)
                # self.trans_base = self.tf2_buffer.lookup_transform('base_link', mouth_frame_id, lookup_time, timeout_ros)

                # x_t = self.trans_base.transform.translation.x
                # y_t = self.trans_base.transform.translation.y
                # z_t = self.trans_base.transform.translation.z
                # marker.pose.position.y is for horizontal z is for depth and x is up and down
                self.y = marker.pose.position.y
                self.x = marker.pose.position.x
                self.z = marker.pose.position.z
                # print('x dist:', x_t)
                # print('y dist:', y_t)
                # print('z dist:', z_t)
                # print()
                # rospy.sleep(rospy.Duration(5))
                print(self.z)

                self.follow_person()


    def follow_person(self):
        command = Twist()
        stopping_dist = 0.3 + 0.5

        if abs(self.z) < stopping_dist:
            print('Stopping PID')        
            command.linear.x = 0.0
            command.linear.y = 0.0
            command.linear.z = 0.0
            command.angular.x = 0.0 
            command.angular.y = 0.0
            command.angular.z = 0.0

            self.pub.publish(command)

            lookup_time = rospy.Time(0) # return most recent transform
            timeout_ros = rospy.Duration(0.1)
            self.align_arm_to_marker(self.mouth_frame_id, lookup_time, timeout_ros)

        else:
            k1 = 0.5
            k2 = 2#1
            
            command.linear.x = k1*self.z - 0.3
            command.linear.y = 0.0
            command.linear.z = 0.0
            command.angular.x = 0.0 
            command.angular.y = 0.0
            command.angular.z = k2*self.y

            self.pub.publish(command)
            
        # self.pub.publish(command)
        


    #Change and Integrate
    def align_arm_to_marker(self, marker_frame_id, lookup_time, timeout_ros):
        # rospy.sleep(rospy.Duration(2))
        # print(len(self.markers))

        #Might need idk
        
        self.switch_base_to_manipulation = rospy.ServiceProxy('/switch_to_position_mode', Trigger)
        self.switch_base_to_manipulation()

        if len(self.mouth_markers) > 0:
            print('Detected Marker')

            for marker in self.mouth_markers:
                if marker.type == self.mouth_marker_type:

                    rospy.loginfo("marker found")

                    # aruco_position = marker.pose.position
                    # self.aruco_point = PointStamped()
                    # self.aruco_point.point = aruco_position
                    # header = self.aruco_point.header
                    # header.stamp = marker.header.stamp
                    # header.frame_id = marker.header.frame_id
                    # header.seq = marker.header.seq

                    # aruco_frame_id = marker.header.frame_id

                    self.trans_base = self.tf2_buffer.lookup_transform('base_link', marker_frame_id, lookup_time, timeout_ros)
                    # print(self.trans_base)                   
                    

                    x_b = self.trans_base.transform.rotation.x
                    y_b = self.trans_base.transform.rotation.y
                    z_b = self.trans_base.transform.rotation.z
                    w_b = self.trans_base.transform.rotation.w

                    x_rot_base, y_rot_base, z_rot_base = euler_from_quaternion([x_b, y_b, z_b, w_b])
                    print('Base')
                    print(x_rot_base, y_rot_base, z_rot_base)
                    print()


                    self.trans_camera = self.tf2_buffer.lookup_transform('camera_link', marker_frame_id, lookup_time, timeout_ros)
                    # print(self.trans_camera)
                    

                    x_c = self.trans_camera.transform.rotation.x
                    y_c = self.trans_camera.transform.rotation.y
                    z_c = self.trans_camera.transform.rotation.z
                    w_c = self.trans_camera.transform.rotation.w


                    x_rot_cam, y_rot_cam, z_rot_cam = euler_from_quaternion([x_c, y_c, z_c, w_c ])
                    print('Camera')
                    print(x_rot_cam, y_rot_cam, z_rot_cam)
                    print()

                    angle_rotate_base = z_rot_base + z_rot_cam + np.radians(35)
                    print('Angle to Rotate Base to:', angle_rotate_base)
                    angle_rotate_camera = self.curr_head_pan - angle_rotate_base
                    print('Angle to Rotate Camera to:', angle_rotate_camera)


                    self.joint_controller.set_cmd(joints=[
                            Joints.joint_lift,
                            Joints.rotate_mobile_base,
                            Joints.joint_head_pan], 
                            values=[
                            0.9, angle_rotate_base, angle_rotate_camera
                            ], wait=True)
                        
                    print('Alignment Done')

                    # return angle_rotate_base, angle_rotate_camera
    
    # def main_function(self):
    #     # for marker in self.mouth_markers:
    #     # if marker.type == self.mouth_marker_type:
    #     if self.mouth_markers[0].type == self.mouth_marker_type:
    #         marker = self.mouth_markers[0]
        
    #         mouth_frame_id = marker.header.frame_id
    #         lookup_time = rospy.Time(0) # return most recent transform
    #         timeout_ros = rospy.Duration(0.1)
    #         self.trans_base = self.tf2_buffer.lookup_transform('base_link', mouth_frame_id, lookup_time, timeout_ros)

    #         x_t = self.trans_base.transform.translation.x
    #         y_t = self.trans_base.transform.translation.y
    #         z_t = self.trans_base.transform.translation.z

    #         print('x dist:', x_t)
    #         print('y dist:', y_t)
    #         print('z dist:', z_t)
    #         print()
    #         rospy.sleep(rospy.Duration(5))

if __name__ == '__main__':
    rospy.init_node('person_detection')
    person_detection = PersonDetectionNode()

    # person_detection.main_function() #COMMENT OUT
    rospy.spin()