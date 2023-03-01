import rospy
import math
import rospy
import time
import actionlib
import os
import csv
from datetime import datetime
import tkinter
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
import tf2_ros
import tf2_geometry_msgs
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from rail_stretch_navigation.srv import GraspAruco
from manipulate_arm import Manipulate_Arm


LIFT_HEIGHT = 0.05
MAX_LIFT = 0.95
MIN_LIFT = 0.30
MAX_WRIST_EXTENSION = 0.5
MIN_WRIST_EXTENSION = 0.0

class ArucoGrasper(object):
    

    def get_bounded_lift(self, lift_value):
        if lift_value > MAX_LIFT:
            return MAX_LIFT
        elif lift_value < MIN_LIFT:
            return MIN_LIFT

        return lift_value

    def get_bounded_extension(self, extension_value):
        if extension_value > MAX_WRIST_EXTENSION:
            return MAX_WRIST_EXTENSION
        elif extension_value < MIN_WRIST_EXTENSION:
            return MIN_WRIST_EXTENSION

        return extension_value

    def __init__(self):
        self.rate = 10
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.markers = []
        self.arm = Manipulate_Arm()

        rospy.Subscriber('/aruco/marker_array', MarkerArray, self.aruco_detected_callback)

    def get_displacement(self, marker):
        try:
            transform = self.tf_buffer.lookup_transform('link_grasp_center', marker.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
            p = PoseStamped()
            p.header.frame_id = marker.header.frame_id
            p.header.stamp = rospy.Time(0)
            p.pose = marker.pose

            p_in_grasp_frame = tf2_geometry_msgs.do_transform_pose(p, transform)

            return p_in_grasp_frame.pose.position

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print('aruco_grasper: ERROR')

    def grasp_aruco(self, aruco_name="unknown"):
        self.switch_base_to_position = rospy.ServiceProxy('/switch_to_position_mode', Trigger)
        self.switch_base_to_position()
        
        for i in range(-3, 3):
            self.arm.issue_command(yaw=0,head_pan=-math.pi / 2 + ((math.pi / 6) * i), head_tilt=-math.pi/6)

            # wait for aruco detection
            rospy.sleep(rospy.Duration(2))
            print("Wait over")
            print("Number of markers found: " + str(len(self.markers)))
            for marker in self.markers:
                if marker.text == aruco_name:
                    height_offset = 0
                    depth_offset = 0
                   

                    
                    
                    displacement = self.get_displacement(marker)
                    
                    print(displacement)
                    joints = self.arm.get_joint_positions()
                    extension = joints['joint_arm_l0']
                    lift = joints['joint_lift']
                    joint_wrist_yaw = joints['joint_wrist_yaw']
                    joint_gripper_finger_right = joints['joint_gripper_finger_right']
                    joint_head_pan = joints['joint_head_pan']
                    joint_head_tilt = joints['joint_head_tilt']

                    self.arm.issue_command(yaw=joint_wrist_yaw,gripper=joint_gripper_finger_right, head_pan=joint_head_pan, head_tilt=joint_head_tilt, 
                                           lift=self.get_bounded_lift(displacement.z + height_offset),extension=self.get_bounded_lift(displacement.x + depth_offset))

                    rospy.sleep(rospy.Duration(2))
                    print("Went to position")

                    return True

    def aruco_detected_callback(self, msg):
        self.markers = msg.markers

if __name__ == '__main__':
    rospy.init_node('aruco_grasper')
    aruco_grasper = ArucoGrasper()
    aruco_grasper.grasp_aruco()
    # rospy.spin()