#!/usr/bin/env python3

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

        # num_pan_angles = 5

        # # looking out along the arm
        # middle_pan_angle = -math.pi/2.0

        # look_around_range = math.pi/3.0
        # min_pan_angle = middle_pan_angle - (look_around_range / 2.0)
        # max_pan_angle = middle_pan_angle + (look_around_range / 2.0)
        # pan_angle = min_pan_angle
        # pan_increment = look_around_range / float(num_pan_angles - 1.0)
        # self.pan_angles = [min_pan_angle + (i * pan_increment)
        #                    for i in range(num_pan_angles)]
        # self.pan_angles = self.pan_angles + self.pan_angles[1:-1][::-1]

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

        self.trans_base = TransformStamped()
        self.trans_camera = TransformStamped()

        self.joint_states_subscriber = rospy.Subscriber('joint_states', JointState, self.joint_state_callback)
        
        self.mouth_position_subscriber = rospy.Subscriber('/nearest_mouth/marker_array', MarkerArray, self.mouth_position_callback)
        

        # with self.move_lock: 
        #     self.handover_goal_ready = False
        
    # def joint_states_callback(self, joint_states):
    #     with self.joint_states_lock: 
    #         self.joint_states = joint_states
    #     wrist_position, wrist_velocity, wrist_effort = hm.get_wrist_state(joint_states)
    #     self.wrist_position = wrist_position
    #     lift_position, lift_velocity, lift_effort = hm.get_lift_state(joint_states)
    #     self.lift_position = lift_position

    # def look_around_callback(self):
    #     # Cycle the head back and forth looking for a person to whom
    #     # to handout the object.
    #     with self.move_lock:
    #         pan_index = (self.prev_pan_index + 1) % len(self.pan_angles)
    #         pan_angle = self.pan_angles[pan_index]
    #         pose = {'joint_head_pan': pan_angle, 'joint_head_tilt': self.tilt_angle}
    #         self.move_to_pose(pose)
    #         self.prev_pan_index = pan_index

    #         if self.mouth_point is not None:
    #             self.look_around = False

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
        rospy.sleep(rospy.Duration(2))
        # print(len(self.markers))

        # if len(self.markers) > 0:
        print('Detected Mouth')

        # for marker in self.markers:
        for marker in self.mouth_markers:
            # if marker.text == aruco_name:
            if marker.type == self.mouth_marker_type:

                rospy.loginfo("marker found")


                self.trans_base = self.tf2_buffer.lookup_transform('base_link', mouth_frame_id, lookup_time, timeout_ros)
                # print(self.trans_base)                   
                

                x_b = self.trans_base.transform.rotation.x
                y_b = self.trans_base.transform.rotation.y
                z_b = self.trans_base.transform.rotation.z
                w_b = self.trans_base.transform.rotation.w

                x_rot_base, y_rot_base, z_rot_base = euler_from_quaternion([x_b, y_b, z_b, w_b])
                print('Base')
                print(x_rot_base, y_rot_base, z_rot_base)
                print()



                self.trans_camera = self.tf2_buffer.lookup_transform('camera_link', mouth_frame_id, lookup_time, timeout_ros)
                # print(self.trans_camera)
                

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
                
                angle_rotate_camera = self.curr_head_pan - angle_rotate_base
                # angle_rotate_camera = self.curr_head_pan - angle_rotate_base
                print('Angle to Rotate Camera to:', angle_rotate_camera)

                # angle_rotate_base = angle_rotate_camera
                # print('Angle to Rotate Base by:', angle_rotate_base)


                # self.trans_camera = self.tf2_buffer.lookup_transform('base_link', 'camera_link', lookup_time, timeout_ros)
                # # print(self.trans_camera)
                

                # x_cb = self.trans_camera.transform.rotation.x
                # y_cb = self.trans_camera.transform.rotation.y
                # z_cb = self.trans_camera.transform.rotation.z
                # w_cb = self.trans_camera.transform.rotation.w


                # x_rot_cb, y_rot_cb, z_rot_cb = euler_from_quaternion([x_cb, y_cb, z_cb, w_cb])
                # print('Camera to Base')
                # print(x_rot_cam, y_rot_cam, z_rot_cam)
                # print()


                return angle_rotate_base, angle_rotate_camera
            


    def move_to_person(self):

        with self.move_lock:

            self.switch_base_to_manipulation = rospy.ServiceProxy('/switch_to_position_mode', Trigger)
            self.switch_base_to_manipulation()

            for i in range(-3, 3):
                rospy.loginfo("Panning")
                
                self.joint_controller.set_cmd(joints=[
                    Joints.joint_wrist_yaw,
                    Joints.joint_head_pan,
                    Joints.joint_head_tilt,
                    Joints.gripper_aperture
                    ],
                    values=[0, -math.pi / 2 + ((math.pi / 6) * i) , -0.1, 0.0], # gripper facing right, camera facing right, camera tilted towards floor, gripper open
                    wait=True)

                # wait for aruco detection

                rospy.sleep(rospy.Duration(2))

                print(len(self.mouth_markers))
                # print(self.mouth_markers) 

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

                        lookup_time = rospy.Time(0) # return most recent transform
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


                        #Align Front to Mouth Marker
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

                        return True
                        # print('Sleeping...')
                        # rospy.sleep(rospy.Duration(10))
                

                        # handoff_object = True

                        # if handoff_object:
                        #     # attempt to handoff the object at a location below
                        #     # the mouth with respect to the world frame (i.e.,
                        #     # gravity)
                        #     target_offset_xyz = np.array([0.0, 0.0, -0.2])
                        # else: 
                        #     object_height_m = 0.1
                        #     target_offset_xyz = np.array([0.0, 0.0, -object_height_m])
                        # target_xyz = mouth_xyz + target_offset_xyz

                        # fingers_error = target_xyz - fingers_xyz
                        # print('fingers_error =', fingers_error)

                        # delta_forward_m = fingers_error[0] 
                        # delta_extension_m = -fingers_error[1]
                        # delta_lift_m = fingers_error[2]

                        # max_lift_m = 1.0
                        # lift_goal_m = self.lift_position + delta_lift_m
                        # lift_goal_m = min(max_lift_m, lift_goal_m)
                        # self.lift_goal_m = lift_goal_m

                        # self.mobile_base_forward_m = delta_forward_m

                        # max_wrist_extension_m = 0.5
                        # wrist_goal_m = self.wrist_position + delta_extension_m

                        # if handoff_object:
                        #     # attempt to handoff the object by keeping distance
                        #     # between the object and the mouth distance
                        #     #wrist_goal_m = wrist_goal_m - 0.3 # 30cm from the mouth
                        #     wrist_goal_m = wrist_goal_m - 0.25 # 25cm from the mouth
                        #     wrist_goal_m = max(0.0, wrist_goal_m)

                        # self.wrist_goal_m = min(max_wrist_extension_m, wrist_goal_m)

                        # self.handover_goal_ready = True

            
    # def trigger_handover_object_callback(self, request):
    #     with self.move_lock: 
    #         # First, retract the wrist in preparation for handing out an object.
    #         pose = {'wrist_extension': 0.005}
    #         self.move_to_pose(pose)

    #         if self.handover_goal_ready: 
    #             pose = {'joint_lift': self.lift_goal_m}
    #             self.move_to_pose(pose)
    #             tolerance_distance_m = 0.01
    #             at_goal = self.move_base.forward(self.mobile_base_forward_m, detect_obstacles=False, tolerance_distance_m=tolerance_distance_m)
    #             pose = {'wrist_extension': self.wrist_goal_m}
    #             self.move_to_pose(pose)
    #             self.handover_goal_ready = False
    #             self.look_around = True

    #         return TriggerResponse(
    #             success=True,
    #             message='Completed object handover!'
    #             )

    
    # def main(self):
    #     # hm.HelloNode.main(self, 'handover_object', 'handover_object', wait_for_first_pointcloud=False)

    #     self.joint_states_subscriber = rospy.Subscriber('joint_states', JointState, self.joint_state_callback)
        
    #     # self.trigger_deliver_object_service = rospy.Service('/deliver_object/trigger_deliver_object',
    #     #                                                     Trigger,
    #     #                                                     self.trigger_handover_object_callback)
        
    #     self.mouth_position_subscriber = rospy.Subscriber('/nearest_mouth/marker_array', MarkerArray, self.mouth_position_callback)
        
        
        # This rate determines how quickly the head pans back and forth.
        # rate = rospy.Rate(0.5)
        # self.look_around = True
        # while not rospy.is_shutdown():
        #     if self.look_around: 
        #         self.look_around_callback()
        #     rate.sleep()


if __name__ == '__main__':
    rospy.init_node('person_detection')
    person_detection = PersonDetectionNode()
    person_detection.move_to_person()
    rospy.spin()



# if __name__ == '__main__':
#     try:
#         parser = ap.ArgumentParser(description='Handover an object.')
#         args, unknown = parser.parse_known_args()
#         rospy.init_node('hand_over_object')
#         node = PersonDetectionNode()
#         node.main()
#         rospy.spin()

#     except KeyboardInterrupt:
#         rospy.loginfo('interrupt received, so shutting down')
