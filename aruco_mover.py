import rospy
import math
import time
import actionlib
import os
import csv
from datetime import datetime
import tkinter
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
# from manipulate_arm import Manipulate_Arm
from geometry_msgs.msg import PointStamped
import ros_numpy as rn
import numpy as np
from std_msgs.msg import Int8

import stretch_body.xbox_controller as xc

LIFT_HEIGHT = 0.05
MAX_LIFT = 0.95
MIN_LIFT = 0.30
MAX_WRIST_EXTENSION = 0.5
MIN_WRIST_EXTENSION = 0.0

class ArucoGrasper(object):
    LIFT_HEIGHT = 0.05

    def get_bounded_lift(self, lift_value):
        if lift_value > JointController.MAX_LIFT:
            return JointController.MAX_LIFT
        elif lift_value < JointController.MIN_LIFT:
            return JointController.MIN_LIFT

        return lift_value

    def get_bounded_extension(self, extension_value):
        if extension_value > JointController.MAX_WRIST_EXTENSION:
            return JointController.MAX_WRIST_EXTENSION
        elif extension_value < JointController.MIN_WRIST_EXTENSION:
            return JointController.MIN_WRIST_EXTENSION

        return extension_value

    def __init__(self):
        self.rate = 10
        # self.grasp_aruco_service = rospy.Service('/aruco_grasper/grasp_aruco', GraspAruco, self.grasp_aruco)
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)
        self.joint_controller = JointController()
        self.markers = []

        self.MM_publisher = rospy.Subscriber('/start_aruco_flag', Int8, self.button_callback)
        # self.buttons = rospy.Subscriber('/button', Int8, self.button_callback)
        # self.buttons

        self.mouth_publisher = rospy.Publisher('/detect_mouth', Int8, queue_size=10)
        self.mouth_flag = 0
        
        self.aruc_marker = rospy.Subscriber('/aruco/marker_array', MarkerArray, self.aruco_detected_callback)

        self.sub = rospy.Subscriber('joint_states', JointState, self.joint_state_callback)

        self.curr_wrist_extension = 0.0
        self.curr_lift = 0.0
        self.curr_yaw = 0.0
        self.curr_gripper = 0.0
        self.curr_head_pan = 0.0
        self.curr_head_tilt = 0.0

        self.trans_base = TransformStamped()
        self.trans_camera = TransformStamped()

        self.xbox_controller = xc.XboxController()
        self.xbox_controller.start()

        self.button_state = 0
        self.do_once = 0

        # self.switch_base_to_manipulation = rospy.ServiceProxy('/switch_to_position_mode', Trigger)
        # self.switch_base_to_manipulation()

        # self.joint_controller.stow() #Ucomment
        # print("Stowed")

    def button_callback(self, msg):
        self.button_state = msg.data
        if self.button_state == 1 and self.do_once == 0:

            self.switch_base_to_manipulation = rospy.ServiceProxy('/switch_to_position_mode', Trigger)
            self.switch_base_to_manipulation()
            self.do_once = 1
            self.joint_controller.stow() #Ucomment
            print("Stowed")
            self.grasp_aruco()

    # def voice_command_callback(self, msg):
    #     self.do_state = msg.data
    #     if self.do_state == 1 and self.do_once == 0:
    #         self.do_once = 1
    #         self.grasp_aruco()


    def aruco_detected_callback(self, msg):
        self.markers = msg.markers

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
    

    def align_arm_to_marker(self, aruco_name, aruco_frame_id, lookup_time, timeout_ros):
        if len(self.markers) > 0:
            print('Detected Marker')

            for marker in self.markers:
                if marker.text == aruco_name:

                    rospy.loginfo("marker found")

                    # aruco_position = marker.pose.position
                    # self.aruco_point = PointStamped()
                    # self.aruco_point.point = aruco_position
                    # header = self.aruco_point.header
                    # header.stamp = marker.header.stamp
                    # header.frame_id = marker.header.frame_id
                    # header.seq = marker.header.seq

                    # aruco_frame_id = marker.header.frame_id

                    self.trans_base = self.tf2_buffer.lookup_transform('base_link', aruco_frame_id, lookup_time, timeout_ros)
                    # print(self.trans_base)                   
                    

                    x_b = self.trans_base.transform.rotation.x
                    y_b = self.trans_base.transform.rotation.y
                    z_b = self.trans_base.transform.rotation.z
                    w_b = self.trans_base.transform.rotation.w

                    x_rot_base, y_rot_base, z_rot_base = euler_from_quaternion([x_b, y_b, z_b, w_b])
                    print('Base')
                    print(x_rot_base, y_rot_base, z_rot_base)
                    print()



                    self.trans_camera = self.tf2_buffer.lookup_transform('camera_link', aruco_frame_id, lookup_time, timeout_ros)
                    # print(self.trans_camera)
                    

                    x_c = self.trans_camera.transform.rotation.x
                    y_c = self.trans_camera.transform.rotation.y
                    z_c = self.trans_camera.transform.rotation.z
                    w_c = self.trans_camera.transform.rotation.w


                    x_rot_cam, y_rot_cam, z_rot_cam = euler_from_quaternion([x_c, y_c, z_c, w_c ])
                    print('Camera')
                    print(x_rot_cam, y_rot_cam, z_rot_cam)
                    print()

                    angle_rotate_base = z_rot_base + z_rot_cam
                    print('Angle to Rotate Base to:', angle_rotate_base)
                    angle_rotate_camera = self.curr_head_pan - angle_rotate_base
                    print('Angle to Rotate Camera to:', angle_rotate_camera)

                    return angle_rotate_base, angle_rotate_camera

                    


    def grasp_aruco(self, aruco_name="target_object"):
    # def grasp_aruco(self, aruco_name="unknown"):

        rospy.sleep(rospy.Duration(0.1))   

        if self.do_once == 1:



            for i in range(-4, 10):
                rospy.loginfo("Panning")
                
                self.joint_controller.set_cmd(joints=[
                    Joints.joint_wrist_yaw,
                    Joints.joint_head_pan,
                    Joints.joint_head_tilt,
                    Joints.gripper_aperture
                    ],
                    values=[0, -2*math.pi / 3 + ((math.pi / 12) * i) , -math.pi/6, 0.0], # gripper facing right, camera facing right, camera tilted towards floor, gripper open
                    wait=True)

                # wait for aruco detection
                rospy.sleep(rospy.Duration(1.5))
                print(len(self.markers))


                for marker in self.markers:
                    print(marker.text)
                    if marker.text == aruco_name:
                        rospy.loginfo("marker found")

                        aruco_position = marker.pose.position
                        self.aruco_point = PointStamped()
                        self.aruco_point.point = aruco_position
                        header = self.aruco_point.header
                        header.stamp = marker.header.stamp
                        header.frame_id = marker.header.frame_id
                        header.seq = marker.header.seq

                        lookup_time = rospy.Time(0) # return most recent transform
                        timeout_ros = rospy.Duration(0.1)

                        print('Current Camera Pan', self.curr_head_pan)
                        print()


                        #Align Arm to Aruco Marker
                        angle_rotate_base, angle_rotate_camera = self.align_arm_to_marker(aruco_name, header.frame_id, lookup_time, timeout_ros)


                        self.joint_controller.set_cmd(joints=[
                            Joints.rotate_mobile_base,
                            Joints.joint_head_pan], 
                            values=[
                            angle_rotate_base, angle_rotate_camera
                            ], wait=True)
                        
                        print('Alignment Done')
                        rospy.sleep(rospy.Duration(2))



                        #Calculate Translation, Lift, Extension
                                            
                        old_frame_id = self.aruco_point.header.frame_id[:]
                        new_frame_id = 'base_link'
                        stamped_transform = self.tf2_buffer.lookup_transform(new_frame_id, old_frame_id, lookup_time, timeout_ros)
                        points_in_old_frame_to_new_frame_mat = rn.numpify(stamped_transform.transform)
                        camera_to_base_mat = points_in_old_frame_to_new_frame_mat

                        grasp_center_frame_id = 'link_grasp_center'
                        stamped_transform = self.tf2_buffer.lookup_transform(new_frame_id, grasp_center_frame_id, lookup_time, timeout_ros)
                        grasp_center_to_base_mat = rn.numpify(stamped_transform.transform)

                        aruco_camera_xyz = np.array([0.0, 0.0, 0.0, 1.0])
                        aruco_camera_xyz[:3] = rn.numpify(self.aruco_point.point)[:3]

                        aruco_xyz = np.matmul(camera_to_base_mat, aruco_camera_xyz)[:3]
                        fingers_xyz = grasp_center_to_base_mat[:,3][:3]

                        handoff_object = True

                        if handoff_object:
                            # attempt to handoff the object at a location below
                            # the aruco with respect to the world frame (i.e.,
                            # gravity)
                            target_offset_xyz = np.array([0.0, 0.0, 0.0])
                        else: 
                            object_height_m = 0.0
                            target_offset_xyz = np.array([0.0, 0.0, -object_height_m])
                        target_xyz = aruco_xyz + target_offset_xyz

                        fingers_error = target_xyz - fingers_xyz
                        print('fingers_error =', fingers_error)

                        delta_forward_m = fingers_error[0] 
                        delta_extension_m = fingers_error[1]# + 0.29
                        delta_lift_m = fingers_error[2]

                        max_lift_m = 1.0
                        lift_goal_m = delta_lift_m + 0.30
                        lift_goal_m = min(max_lift_m, lift_goal_m)

                        #Translate Mobile Base
                        self.joint_controller.set_cmd(joints=[
                                Joints.joint_lift,
                                Joints.translate_mobile_base,
                                Joints.gripper_aperture
                            ],
                            #self.joint_controller.joint_states.position[Joints.joint_lift.value] + 
                            values=[
                                lift_goal_m,    
                                delta_forward_m, 
                                0.06
                            ],
                            wait=True)
                        


                        max_wrist_extension_m = 0.5
                    
                        wrist_goal_m = self.curr_wrist_extension + abs(delta_extension_m)
                        
                        print('difference', abs(max_wrist_extension_m - abs(delta_extension_m)))

                        # if handoff_object:
                            # attempt to handoff the object by keeping distance
                            # between the object and the aruco distance
                            #wrist_goal_m = wrist_goal_m - 0.3 # 30cm from the aruco

                            #Extension offset should only be applied if the marker is further away from the camera
                            #Offset should be zero if marker is closer than 0.5 meters away from camera.

                        if abs(max_wrist_extension_m - abs(delta_extension_m)) > 0.35:
                            wrist_goal_m = wrist_goal_m + 0.3 # 25cm from the aruco
                        else:
                            wrist_goal_m += 0.03

                        wrist_goal_m = max(0.0, wrist_goal_m)

                        print('extension goal', wrist_goal_m)

                        wrist_goal_m = min(max_wrist_extension_m, wrist_goal_m)
                        print(wrist_goal_m, self.curr_wrist_extension, delta_extension_m)

                        

                        #Grab Object
                        self.joint_controller.set_cmd(joints=[Joints.wrist_extension],
                            values=[wrist_goal_m],
                            wait=True)
                        
                        rospy.sleep(rospy.Duration(2))

                        self.joint_controller.set_cmd(joints=[Joints.gripper_aperture], values=[0.0], wait=True)



                        #Arm Position to Hold Object
                        self.joint_controller.set_cmd(joints=[
                                Joints.joint_lift, Joints.wrist_extension
                            ],
                            values=[
                                .95,0.0
                            ],
                            wait=True)
                        

                        self.joint_controller.set_cmd(joints=[Joints.joint_wrist_yaw], values=[math.pi], wait=True)
                        
                        # self.joint_controller.stow()
                        self.mouth_flag = 1
                        self.mouth_publisher.publish(self.mouth_flag)
                        
                        rospy.signal_shutdown("Completed")





if __name__ == '__main__':
    rospy.init_node('aruco_grasper')
    aruco_grasper = ArucoGrasper()
    # aruco_grasper.button_callback(Int8(1))
    rospy.spin()
