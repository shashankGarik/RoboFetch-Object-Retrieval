#!/usr/bin/env python3
import rospy
import math

import tf2_ros
import tf2_geometry_msgs
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from joint_controller import JointController, Joints
from visualization_msgs.msg import MarkerArray
from rail_stretch_navigation.srv import GraspAruco
from std_msgs.msg import Int8

import sys
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import hello_helpers.hello_misc as hm
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
# from nav_robo import StretchNavigation

import rospy
import actionlib
import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Int8
from tf import transformations
from manipulate_arm import Manipulate_Arm
import numpy as np

class StretchNavigation:
    """
    A simple encapsulation of the navigation stack for a Stretch robot.
    """
    def __init__(self):
        """
        Create an instance of the simple navigation interface.
        :param self: The self reference.
        """
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo('{0}: Made contact with move_base server'.format(self.__class__.__name__))

        self.nav_state = 1
        self.flag_mm = 0
        self.flag_nav = False

        

        # self.nav_subscriber = rospy.Subscriber('/nav_flag', Int8, self.nav_callback)
        # self.button_subscriber = rospy.subscriber('/button_flag', Int8, self.button_callback)
        self.MM_publisher = rospy.Publisher('/MM_flag', Int8, queue_size = 50)

        self.voice_command_flag = 0
        self.voice_subscriber = rospy.Subscriber('/voice_command', Int8, self.voice_callback)

        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time()

        self.goal.target_pose.pose.position.x = 0.0
        self.goal.target_pose.pose.position.y = 0.0
        self.goal.target_pose.pose.position.z = 0.0
        self.goal.target_pose.pose.orientation.x = 0.0
        self.goal.target_pose.pose.orientation.y = 0.0
        self.goal.target_pose.pose.orientation.z = 0.0
        self.goal.target_pose.pose.orientation.w = 1.0

        # self.node = Manipulate_Arm()
        # self.grasper = ArucoGrasper()
        

        # self._odom_subscriber = self.create_subscription(Odometry, '/odom', self._motion_controller, 1)
        # self._odom_subscriber


    def voice_callback(self, data):
        self.voice_command_flag = data

    def get_quaternion(self,theta):
        """
        A function to build Quaternians from Euler angles. Since the Stretch only
        rotates around z, we can zero out the other angles.
        :param theta: The angle (radians) the robot makes with the x-axis.
        """
        return Quaternion(*transformations.quaternion_from_euler(0.0, 0.0, theta))

    def go_to(self, x, y, theta):
        """
        Drive the robot to a particular pose on the map. The Stretch only needs
        (x, y) coordinates and a heading.
        :param x: x coordinate in the map frame.
        :param y: y coordinate in the map frame.
        :param theta: heading (angle with the x-axis in the map frame)
        """
        rospy.loginfo('{0}: Heading for ({1}, {2}) at {3} radians'.format(self.__class__.__name__,
        x, y, theta))

        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y
        self.goal.target_pose.pose.orientation = self.get_quaternion(theta)

        self.client.send_goal(self.goal, done_cb=self.done_callback)
        self.client.wait_for_result()
        rospy.loginfo("Going sleeping enjoying")

    def done_callback(self, status, result):
        """
        The done_callback function will be called when the joint action is complete.
        :param self: The self reference.
        :param status: status attribute from MoveBaseActionResult message.
        :param result: result attribute from MoveBaseActionResult message.
        """
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo('{0}: SUCCEEDED in reaching the goal.'.format(self.__class__.__name__))
        else:
            rospy.loginfo('{0}: FAILED in reaching the goal.'.format(self.__class__.__name__))

    # def button_callback(self, buttonpress):
    #     # if buttonpress == 1:
    #         self.nav_state = buttonpress

    def nav_callback(self, lilnav):
        self.flag_nav = lilnav

    def gotoGoal(self):
        # nav = StretchNavigation()
        self.go_to(1.0, 0.0, math.pi / 2)
        # rospy.sleep(5)
        rospy.loginfo("Reached Goal")
        self.flag_mm = 1
        return True
            
    def gotoHome(self):

        self.switch_base_to_navigation = rospy.ServiceProxy('/switch_to_navigation_mode', Trigger)
        self.switch_base_to_navigation()

        self.nav_state == 0
        self.go_to(-0.75, 0.0, 0.0)
        rospy.loginfo("Reached Home")
        self.flag_nav = True
        return True
            # self.flag_nav = 0

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
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.joint_controller = JointController()
        self.markers = []

        self.joint_controller.stow()
        
        rospy.Subscriber('/aruco/marker_array', MarkerArray, self.aruco_detected_callback)
        self.gohomeflag = 0

        self.mm_flag = 1
        # self.MM_subscriber = rospy.Subscriber('/MM_flag', Int8, self.mm_callback)

        


    def mm_callback(self, data):
        self.mm_flag = data

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
                values=[0, -math.pi / 2 + ((math.pi / 6) * i) , -math.pi/6, 0.0], # gripper facing right, camera facing right, camera tilted towards floor, gripper open
                wait=True)

            # wait for aruco detection
            rospy.sleep(rospy.Duration(2))

            for marker in self.markers:
                if marker.text == aruco_name:
                    rospy.loginfo("marker found")
                    #  'height_offset': 0.05
                    # 'depth_offset': 0.04
                    height_offset = 0.05 #rospy.get_param('/aruco_marker_info/{}/height_offset'.format(marker.id), default=0)
                    depth_offset = 0.04 #rospy.get_param('/aruco_marker_info/{}/depth_offset'.format(marker.id), default=0)
                    

                    displacement = self.get_displacement(marker)
                    print(displacement)
                    print(self.joint_controller.joint_states.position)
                    # print(self.joint_controller.joint_states.position[Joints.joint_lift.value] + displacement.z + height_offset)
                    # print(self.joint_controller.joint_states.position[Joints.wrist_extension.value])
                    # print(self.joint_controller.joint_states.position[Joints.wrist_extension.value] + displacement.x + depth_offset)

                    self.joint_controller.set_cmd(joints=[
                            Joints.joint_lift,
                            Joints.translate_mobile_base,
                            Joints.gripper_aperture
                        ],
                        #self.joint_controller.joint_states.position[Joints.joint_lift.value] + 
                        values=[
                            self.get_bounded_lift(displacement.z + height_offset),    
                            displacement.y, 
                            0.06
                        ],
                        wait=True)

                    # self.joint_controller.set_cmd(joints=[Joints.wrist_extension],
                    #     values=[self.get_bounded_extension(JointController.MAX_WRIST_EXTENSION/2 + displacement.x + depth_offset)],
                    #     wait=True)

                    self.joint_controller.set_cmd(joints=[Joints.wrist_extension],
                        values=[self.get_bounded_extension(0.356)],
                        wait=True)

                    self.joint_controller.set_cmd(joints=[Joints.gripper_aperture], values=[0.0], wait=True)
                    
                    self.joint_controller.set_cmd(joints=[
                            Joints.joint_lift, Joints.wrist_extension
                        ],
                        values=[
                            0.9,0.1
                        ],
                        wait=True)
                    

                    self.joint_controller.set_cmd(joints=[Joints.joint_wrist_yaw], values=[math.pi], wait=True)
                    
                    # self.joint_controller.stow()

                    

                    self.gohomeflag = 1


                    # print(self.joint_controller.joint_states.position)
                    return True

    
    
    def aruco_detected_callback(self, msg):
        self.markers = msg.markers

class GetVoiceCommands:
    """
    A class that subscribes to the speech to text recognition messages, prints
    a voice command menu, and defines step size for translational and rotational
    mobile base motion.
    """
    def __init__(self):
        """
        A function that initializes subscribers and defines the three different
        step sizes.
        :param self: The self reference.
        """
        # Initialize the voice command
        self.voice_command = None
        self.rad_per_deg = math.pi/180.0

        # Initialize the sound direction
        self.sound_direction = 0

        # Initialize subscribers
        self.speech_to_text_sub  = rospy.Subscriber("/speech_to_text",  SpeechRecognitionCandidates, self.callback_speech)
        self.sound_direction_sub = rospy.Subscriber("/sound_direction", Int32,                       self.callback_direction)
        
        self.voice_command_flag = 0
        self.voice_command_sent = rospy.Publisher('/voice_command', Int8, queue_size = 50)

    def callback_direction(self, msg):
        """
        A callback function that converts the incoming message, sound direction,
        from degrees to radians.
        :param self: The self reference.
        :param msg: The Int32 message type that represents the sound direction.
        """
        self.sound_direction = msg.data * -self.rad_per_deg

    def callback_speech(self,msg):
        """
        A callback function takes the incoming message, a list of the speech to
        text, and joins all items in that iterable list into a single string.
        :param self: The self reference.
        :param msg: The SpeechRecognitionCandidates message type.
        """
        self.voice_command = ' '.join(map(str,msg.transcript))

    def print_commands(self):
        """
        A function that prints the voice teleoperation menu.
        :param self: The self reference.
        """
        print('Say help')

    def get_command(self):
        """
        A function that defines the teleoperation command based on the voice command.
        :param self: The self reference.

        :returns command: A dictionary type that contains the type of base motion.
        """
        command = None
        # Move base forward command
        
        if self.voice_command == 'help':
            print(self.voice_command)
            command = True
            
            print("Command recieved")
            
            # self.voice_command_flag = 1
        
        else:
            # self.voice_command_flag = 0
            command = False

        # Reset voice command to None
        self.voice_command = None

        return command

        # self.voice_command_sent.publish(self.voice_command_flag)

            # navv = StretchNavigation()
            # navv.gotoGoal()
            

        

        # return the updated command
        # return self.voice_command_flag  

class VoiceTeleopNode(hm.HelloNode):
    """
    A class that inherits the HelloNode class from hm and sends joint trajectory
    commands.
    """
    def __init__(self):
        """
        A function that declares object from the GetVoiceCommands class, instantiates
        the HelloNode class, and set the publishing rate.
        :param self: The self reference.
        """
        hm.HelloNode.__init__(self)
        self.rate = 10.0
        self.joint_state = None
        self.speech = GetVoiceCommands()

        # self.nav_publisher = rospy.Publisher('nav_flag', Int8, queue_size = 50)
        self.nav = StretchNavigation()
        self.grasper = ArucoGrasper()
        self.nav_flag = 0


    def joint_states_callback(self, msg):
        """
        A callback function that stores Stretch's joint states.
        :param self: The self reference.
        :param msg: The JointState message type.
        """
        self.joint_state = msg

        # status = self.grasper.grasp_aruco()
        # if status == True:
        #     self.nav_flag = 1
        #     self.nav_publisher.publish(self.nav_flag)


    def send_command(self, command):
        """
        Function that makes an action call and sends base joint trajectory goals.
        :param self: The self reference.
        :param command: A dictionary that contains the base motion type and increment size.
        """
        joint_state = self.joint_state
        # Conditional statement to send  joint trajectory goals
        if (joint_state is not None) and (command is not None):
            # Assign point as a JointTrajectoryPoint message type
            point = JointTrajectoryPoint()
            point.time_from_start = rospy.Duration(0.0)

            # Assign trajectory_goal as a FollowJointTrajectoryGoal message type
            trajectory_goal = FollowJointTrajectoryGoal()
            trajectory_goal.goal_time_tolerance = rospy.Time(1.0)

            # Extract the joint name from the command dictionary
            joint_name = command['joint']
            trajectory_goal.trajectory.joint_names = [joint_name]

            # Extract the increment type from the command dictionary
            inc = command['inc']
            rospy.loginfo('inc = {0}'.format(inc))
            new_value = inc

            # Assign the new_value position to the trajectory goal message type
            point.positions = [new_value]
            trajectory_goal.trajectory.points = [point]
            trajectory_goal.trajectory.header.stamp = rospy.Time.now()
            rospy.loginfo('joint_name = {0}, trajectory_goal = {1}'.format(joint_name, trajectory_goal))

            # Make the action call and send goal of the new joint position
            self.trajectory_client.send_goal(trajectory_goal)
            rospy.loginfo('Done sending command.')

            # Reprint the voice command menu
            # self.speech.print_commands()

    def main(self):
        """
        The main function that instantiates the HelloNode class, initializes the subscriber,
        and call other methods in both the VoiceTeleopNode and GetVoiceCommands classes.
        :param self: The self reference.
        """
        hm.HelloNode.main(self, 'voice_teleop', 'voice_teleop', wait_for_first_pointcloud=False)
        rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)
        rate = rospy.Rate(self.rate)
        self.speech.print_commands()

        while not rospy.is_shutdown():
            # Get voice command
            command = self.speech.get_command()
            if command:
                print("Going to navigation")

                # if self.grasper.gohomeflag == 0:
                self.grasper.grasp_aruco()

                if self.grasper.gohomeflag == 1:
                    print('Going Home')
                    self.nav.gotoHome()

            # rate.sleep()

            # nav_status = self.nav.gotoGoal()
            # if nav_status:
        

        


        # if grip_status:
        #     self.nav.gotoHome()
        # status = self.grasper.grasp_aruco()

            # Send voice command for joint trajectory goals
            # self.send_command(command)
            # rate.sleep()

if __name__ == '__main__':
    rospy.init_node('voice_teleop')

    try:
        # Instanstiate a `VoiceTeleopNode()` object and execute the main() method
        node = VoiceTeleopNode()
        node.main()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')




# if __name__ == '__main__':
#     rospy.init_node('aruco_grasper')
#     aruco_grasper = ArucoGrasper()
#     aruco_grasper.grasp_aruco()
#     rospy.spin()