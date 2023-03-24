#!/usr/bin/env python3

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
        self.flag_nav = 0

        self.nav_subscriber = rospy.Subscriber('/nav_flag', Int8, self.nav_callback)
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

        self.node = Manipulate_Arm()

        # self._odom_subscriber = self.create_subscription(Odometry, '/odom', self._motion_controller, 1)
        # self._odom_subscriber


    def voice_callback(self, data):
        self.voice_command_flag = data

        if self.voice_command_flag == 1:
            self.gotoGoal()

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

        self.nav_state = 1

        if self.nav_state == 1 and self.flag_nav == 0:
            self.go_to(0.0, 0.0, 0.0)
            rospy.sleep(2)
            self.flag_mm = 1
            self.MM_publisher.publish(self.flag_mm)
            print("flag_mm", self.flag_mm)
            # self.node.issue_command2(lift=0.8, extension=0.5, head_pan=1.7, head_tilt=0.0)
            # rospy.sleep(10)
            # self.flag_nav = 1
            # self.nav_state = 0

        if self.flag_nav == 1:
            self.go_to(-1.0, 0.0, 0.0)
            # self.flag_nav = 0


if __name__ == '__main__':
    rospy.init_node('navigation', argv=sys.argv)

    navv = StretchNavigation()
    rospy.spin()
    # navv.gotoGoal()
    