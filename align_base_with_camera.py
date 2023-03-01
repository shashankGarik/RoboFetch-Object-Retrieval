#!/usr/bin/env python3

import rospy
import actionlib
import sys
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from tf import transformations

from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger


class Move:
    """
    A class that sends Twist messages to move the Stretch robot forward.
    """
    def __init__(self):
        """
        Function that initializes the subscriber.
        :param self: The self reference.
        """
        self.pub = rospy.Publisher('/stretch/cmd_vel', Twist, queue_size=1) #/stretch_diff_drive_controller/cmd_vel for gazebo

        self.switch_base_to_navigation = rospy.ServiceProxy('/switch_to_navigation_mode', Trigger)
        self.switch_base_to_navigation()

    def move_forward(self):
        """
        Function that publishes Twist messages
        :param self: The self reference.

        :publishes command: Twist message.
        """
        command = Twist()
        command.linear.x = 0.0
        command.linear.y = 0.0
        command.linear.z = 0.0
        command.angular.x = 0.0
        command.angular.y = 0.0
        command.angular.z = 0.1
        self.pub.publish(command)

if __name__ == '__main__':
    rospy.init_node('move')
    base_motion = Move()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        base_motion.move_forward()
        rate.sleep()





# class StretchNavigation:
#     """
#     A simple encapsulation of the navigation stack for a Stretch robot.
#     """
#     def __init__(self):
#         """
#         Create an instance of the simple navigation interface.
#         :param self: The self reference.
#         """
#         self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
#         self.client.wait_for_server()
#         rospy.loginfo('{0}: Made contact with move_base server'.format(self.__class__.__name__))

#         self.switch_base_to_navigation = rospy.ServiceProxy('/switch_to_navigation_mode', Trigger)
#         self.switch_base_to_navigation()

#         self.goal = MoveBaseGoal()
#         self.goal.target_pose.header.frame_id = 'map'
#         self.goal.target_pose.header.stamp = rospy.Time()

#         self.goal.target_pose.pose.position.x = 0.0
#         self.goal.target_pose.pose.position.y = 0.0
#         self.goal.target_pose.pose.position.z = 0.0
#         self.goal.target_pose.pose.orientation.x = 0.0
#         self.goal.target_pose.pose.orientation.y = 0.0
#         self.goal.target_pose.pose.orientation.z = 0.0
#         self.goal.target_pose.pose.orientation.w = 1.0

#     def get_quaternion(self,theta):
#         """
#         A function to build Quaternians from Euler angles. Since the Stretch only
#         rotates around z, we can zero out the other angles.
#         :param theta: The angle (radians) the robot makes with the x-axis.
#         """
#         return Quaternion(*transformations.quaternion_from_euler(0.0, 0.0, theta))

#     def go_to(self, x, y, theta):
#         """
#         Drive the robot to a particular pose on the map. The Stretch only needs
#         (x, y) coordinates and a heading.
#         :param x: x coordinate in the map frame.
#         :param y: y coordinate in the map frame.
#         :param theta: heading (angle with the x-axis in the map frame)
#         """
#         rospy.loginfo('{0}: Heading for ({1}, {2}) at {3} radians'.format(self.__class__.__name__,
#         x, y, theta))

#         self.goal.target_pose.pose.position.x = x
#         self.goal.target_pose.pose.position.y = y
#         self.goal.target_pose.pose.orientation = self.get_quaternion(theta)

#         self.client.send_goal(self.goal, done_cb=self.done_callback)

#         state = self.client.send_goal_and_wait(self.goal, execute_timeout=rospy.Duration(200), preempt_timeout=rospy.Duration(200))

#         if state == actionlib.GoalStatus.SUCCEEDED:
#             print(True)

#         # self.client.wait_for_result()

#     def done_callback(self, status, result):
#         """
#         The done_callback function will be called when the joint action is complete.
#         :param self: The self reference.
#         :param status: status attribute from MoveBaseActionResult message.
#         :param result: result attribute from MoveBaseActionResult message.
#         """
#         if status == actionlib.GoalStatus.SUCCEEDED:
#             rospy.loginfo('{0}: SUCCEEDED in reaching the goal.'.format(self.__class__.__name__))
#         else:
#             rospy.loginfo('{0}: FAILED in reaching the goal.'.format(self.__class__.__name__))

# if __name__ == '__main__':
#     rospy.init_node('navigation', argv=sys.argv)
#     nav = StretchNavigation()
#     nav.go_to(0.0, 0.0, np.pi/6)

