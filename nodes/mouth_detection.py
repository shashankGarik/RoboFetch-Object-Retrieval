#!/usr/bin/env python3

#NEED TO COMMENT OUT 2 LINES FOR PUBS AND SUBS TO WORK

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
        self.move_lock = threading.Lock()

        self.curr_wrist_extension = 0.0
        self.curr_lift = 0.0
        self.curr_yaw = 0.0
        self.curr_gripper = 0.0
        self.curr_head_pan = 0.0
        self.curr_head_tilt = 0.0

        self.mouth_markers = []

        self.move_flag = 0

        self.trans_base = TransformStamped()
        self.trans_camera = TransformStamped()

        self.joint_states_subscriber = rospy.Subscriber('joint_states', JointState, self.joint_state_callback)
        
        self.mouth_position_subscriber = rospy.Subscriber('/nearest_mouth/marker_array', MarkerArray, self.mouth_position_callback)
        
        self.detect_mouth = rospy.Subscriber('/detect_mouth', Int8, self.detect_mouth_callback) 

        self.move_publisher = rospy.Publisher('/move_to_person', Int8, queue_size=10)
        
    def detect_mouth_callback(self, mouth_flag):
        if mouth_flag.data == 1:
            self.move_to_person()
        

    def joint_state_callback(self, msg):
        """
        Callback function to deal with the incoming JointState messages.
        :param self: The self reference.
        :param msg: The JointState message.
        """
        self.joint_states = msg
        curr_joint_pos = self.get_joint_positions()
        self.curr_wrist_extension = curr_joint_pos['joint_arm_l0']*4
        self.curr_lift = curr_joint_pos['joint_lift']
        self.curr_yaw = curr_joint_pos['joint_wrist_yaw']
        self.curr_gripper = curr_joint_pos['joint_gripper_finger_right']
        self.curr_head_pan = curr_joint_pos['joint_head_pan']
        self.curr_head_tilt = curr_joint_pos['joint_head_tilt']

    def get_joint_positions(self):
        joints = {}
        for i in range(len(self.joint_states.name)):
            j_name = self.joint_states.name[i]
            joints[j_name] = self.joint_states.position[i]
        return joints
    
    def mouth_position_callback(self, msg):
        self.mouth_markers = msg.markers
    

    def align_front_to_marker(self, mouth_frame_id, lookup_time, timeout_ros):
        print('Detected Mouth')

        for marker in self.mouth_markers:
            if marker.type == self.mouth_marker_type:
                rospy.loginfo("marker found")
                self.trans_base = self.tf2_buffer.lookup_transform('base_link', mouth_frame_id, lookup_time, timeout_ros)

                x_b = self.trans_base.transform.rotation.x
                y_b = self.trans_base.transform.rotation.y
                z_b = self.trans_base.transform.rotation.z
                w_b = self.trans_base.transform.rotation.w

                x_rot_base, y_rot_base, z_rot_base = euler_from_quaternion([x_b, y_b, z_b, w_b])
                print('Base')
                print(x_rot_base, y_rot_base, z_rot_base)
                print()
                self.trans_camera = self.tf2_buffer.lookup_transform('camera_link', mouth_frame_id, lookup_time, timeout_ros)

                x_c = self.trans_camera.transform.rotation.x
                y_c = self.trans_camera.transform.rotation.y
                z_c = self.trans_camera.transform.rotation.z
                w_c = self.trans_camera.transform.rotation.w


                x_rot_cam, y_rot_cam, z_rot_cam = euler_from_quaternion([x_c, y_c, z_c, w_c])
                print('Camera')
                print(x_rot_cam, y_rot_cam, z_rot_cam)
                print()

                total_z_rotation = z_rot_base + z_rot_cam
                angle_rotate_base = total_z_rotation - np.pi/4 
                print('Angle to Rotate Base to:', angle_rotate_base)
                print('Current camera angle', self.curr_head_pan)

                angle_rotate_camera = np.radians(10)
                print('Angle to Rotate Camera to:', angle_rotate_camera)
                
                return angle_rotate_base, angle_rotate_camera
            


    def move_to_person(self):

        with self.move_lock:
            for i in range(-3, 10):
                rospy.loginfo("Panning")
                
                self.joint_controller.set_cmd(joints=[
                    Joints.joint_wrist_yaw,
                    Joints.joint_head_pan,
                    Joints.joint_head_tilt,
                    Joints.gripper_aperture
                    ],
                    values=[math.pi, -math.pi / 2 + ((math.pi / 6) * i) , -0.1, 0.0], # gripper facing right, camera facing right, camera tilted towards floor, gripper open
                    wait=True)

                # wait for aruco detection
                rospy.sleep(rospy.Duration(1.5))
                print(len(self.mouth_markers))

                for marker in self.mouth_markers:
                    if marker.type == self.mouth_marker_type:
                        mouth_position = marker.pose.position
                        self.mouth_point = PointStamped()
                        self.mouth_point.point = mouth_position
                        header = self.mouth_point.header
                        header.stamp = marker.header.stamp
                        header.frame_id = marker.header.frame_id
                        header.seq = marker.header.seq
                        print('******* new mouth point received *******')

                        lookup_time = rospy.Time(0) 
                        timeout_ros = rospy.Duration(0.1)

                        old_frame_id = self.mouth_point.header.frame_id[:]
                        new_frame_id = 'base_link'
                        stamped_transform = self.tf2_buffer.lookup_transform(new_frame_id, old_frame_id, lookup_time, timeout_ros)
                        points_in_old_frame_to_new_frame_mat = rn.numpify(stamped_transform.transform)
                        camera_to_base_mat = points_in_old_frame_to_new_frame_mat

                        grasp_center_frame_id = 'link_grasp_center'
                        stamped_transform = self.tf2_buffer.lookup_transform(new_frame_id, grasp_center_frame_id, lookup_time, timeout_ros)
                        grasp_center_to_base_mat = rn.numpify(stamped_transform.transform)

                        mouth_camera_xyz = np.array([0.0, 0.0, 0.0, 1.0])
                        mouth_camera_xyz[:3] = rn.numpify(self.mouth_point.point)[:3]

                        mouth_xyz = np.matmul(camera_to_base_mat, mouth_camera_xyz)[:3]
                        fingers_xyz = grasp_center_to_base_mat[:,3][:3]

                        angle_rotate_base, angle_rotate_camera = self.align_front_to_marker(marker.header.frame_id, lookup_time, timeout_ros)

                        self.joint_controller.set_cmd(joints=[
                            Joints.rotate_mobile_base], 
                            values=[
                            angle_rotate_base], wait=True)
                        

                        self.joint_controller.set_cmd(joints=[
                            Joints.joint_head_pan], 
                            values=[
                            angle_rotate_camera], wait=True)
                        
                        print('Alignment Done')

                        self.move_flag = 1

                        self.move_publisher.publish(self.move_flag)

                        rospy.signal_shutdown("Completed")

if __name__ == '__main__':
    rospy.init_node('person_detection')
    person_detection = PersonDetectionNode()
    # person_detection.move_to_person()
    rospy.spin()



