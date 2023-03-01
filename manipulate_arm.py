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
import hello_helpers.hello_misc as hm
import gtts
from std_srvs.srv import Trigger
from playsound import playsound
import pyttsx3

class Manipulate_Arm(hm.HelloNode):

    def __init__(self):

        hm.HelloNode.__init__(self)
        self.speechEngine = pyttsx3.init()
        self.sub = rospy.Subscriber('joint_states', JointState, self.callback)
        self.trajectory_goal = FollowJointTrajectoryGoal()
        self.trajectory_client = actionlib.SimpleActionClient('/stretch_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.point = JointTrajectoryPoint()
        # 'joint_lift'[0.1-1.0]',
        # 'wrist_extension'[0-0.51],
        # 'joint_wrist_yaw[0-4.5]', 
        # 'joint_gripper_finger_right/left[0-0.2]'
        # 'joint_head_pan': -4.0->1.75,
        # 'joint_head_tilt': [-1.9->0.4]

        # self.joints = ['wrist_extension', 'joint_lift', 'joint_wrist_yaw'] 
        # head_pose = {'joint_head_pan': 1, 'joint_head_tilt': -1.0}
        # self.initial_pose = {'wrist_extension': 0.2,
        #                 'joint_lift': 0.1,
        #                 'joint_wrist_yaw': 0.0,
        #                 'gripper_aperture': 0.125}
        # self.joint_effort = []
        # self.save_path = '/home/hello-robot/catkin_ws/src/robo_fetch/src'
        # self.export_data = export_data


    def move_to_pose(self, pose):
        # Prepare and send a goal pose to which the robot should move.
        joint_names = [key for key in pose]
        self.trajectory_goal.trajectory.joint_names = joint_names
        joint_positions = [pose[key] for key in joint_names]
        self.point.positions = joint_positions
        self.trajectory_goal.trajectory.points = [self.point]
        self.trajectory_goal.trajectory.header.stamp = rospy.Time.now()
        self.trajectory_client.send_goal(self.trajectory_goal)
        self.trajectory_client.wait_for_result()


    def move_to_initial_configuration(self, extension, lift, yaw, gripper, head_pan, head_tilt):
        # initial_pose = {'wrist_extension': 0.2,
        #                 'joint_wrist_yaw': 4.0}

        pose = {'wrist_extension': extension, 'joint_lift': lift, 'joint_wrist_yaw': yaw, 'joint_gripper_finger_right': gripper, 'joint_head_pan': head_pan, 'joint_head_tilt': head_tilt}


        rospy.loginfo('Move to the initial configuration.')
        self.move_to_pose(pose)



    def callback(self, msg):
        """
        Callback function to deal with the incoming JointState messages.
        :param self: The self reference.
        :param msg: The JointState message.
        """
        self.joint_states = msg


    def issue_command(self,extension=0.0, lift=0.5, yaw=0.0, gripper=0.2, head_pan=-3.0, head_tilt=-1.0):
        """
        Function that makes an action call and sends joint trajectory goals
        to a single joint.
        :param self: The self reference.
        """
        # trajectory_goal = FollowJointTrajectoryGoal()
        # trajectory_goal.trajectory.joint_names = self.joints

        # point0 = JointTrajectoryPoint()
        # point0.positions = [0, 0.5, 0, 0.125]

        # self.move_to_pose(self.initial_pose)
        # extension, lift, yaw, gripper = 0.0, 0.2, 0.3, 0.0
        # self.move_to_initial_configuration(extension, lift, yaw, gripper)

        # time.sleep(2)
        # extension, lift, yaw, gripper = 0.0, 0.5, 0.0, 0.2
        
        
        text = "Command receieved"
        self.speechEngine.say(text)

        # if gripper > 0:
        #     self.speechEngine.say("Opening Gripper")
        # else:
        #     self.speechEngine.say("Closing Gripper")
        self.speechEngine.runAndWait()

        self.move_to_initial_configuration(extension, lift, yaw, gripper, head_pan, head_tilt)
        

        # trajectory_goal.trajectory.points = [point0]
        # trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
        # trajectory_goal.trajectory.header.frame_id = 'arm_l0'
        # # self.trajectory_client.send_goal(trajectory_goal, feedback_cb=self.feedback_callback, done_cb=self.done_callback)
        # self.trajectory_client.send_goal(trajectory_goal, done_cb=self.done_callback)

        
        # rospy.loginfo('Sent position goal = {0}'.format(self.trajectory_goal))
        # self.trajectory_client.wait_for_result()

    def get_joints(self):
        joints = {}
        for i in range(len(self.joint_states.name)):
            j_name = self.joint_states.name[i]
            joints[j_name] = {}
            joints[j_name]["position"] = self.joint_states.position[i]
            joints[j_name]["velocity"] = self.joint_states.velocity[i]
            joints[j_name]["effort"] = self.joint_states.effort[i]
        return joints

    def get_joint_positions(self):
        joints = {}
        for i in range(len(self.joint_states.name)):
            j_name = self.joint_states.name[i]
            joints[j_name] = self.joint_states.position[i]
        return joints

    # def feedback_callback(self,feedback):
    #     """
    #     The feedback_callback function deals with the incoming feedback messages
    #     from the trajectory_client. Although, in this function, we do not use the
    #     feedback information.
    #     :param self: The self reference.
    #     :param feedback: FollowJointTrajectoryActionFeedback message.
    #     """
    #     if 'wrist_extension' in self.joints:
    #         self.joints.remove('wrist_extension')
    #         self.joints.append('joint_arm_l0')

    #     current_effort = []
    #     for joint in self.joints:
    #         index = self.joint_states.name.index(joint)
    #         current_effort.append(self.joint_states.effort[index])

    #     if not self.export_data:
    #         print("name: " + str(self.joints))
    #         print("effort: " + str(current_effort))
    #     else:
    #         self.joint_effort.append(current_effort)


    def done_callback(self, status, result):
        """
        The done_callback function will be called when the joint action is complete.
        Within this function we export the data to a .txt file in  the /stored_data directory.
        :param self: The self reference.
        :param status: status attribute from FollowJointTrajectoryActionResult message.
        :param result: result attribute from FollowJointTrajectoryActionResult message.
        """
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo('Succeeded')
        else:
            rospy.loginfo('Failed')

    def main(self):
        """
        Function that initiates the issue_command function.
        :param self: The self reference.
        """
        # self.switch_base_to_position = rospy.ServiceProxy('/switch_to_position_mode', Trigger)
        # self.switch_base_to_position()
        
        hm.HelloNode.main(self, 'issue_command', 'issue_command', wait_for_first_pointcloud=False)
        rospy.loginfo('issuing command...')
        top = tkinter.Tk()
        B = tkinter.Button(top, text ="Click to go to Home Pose", command = self.issue_command)
        B2 = tkinter.Button(top, text ="Click to go to Pose2", command = lambda: self.issue_command(0.0,0.9,3.0,0.0, 0.0, 0.0))
        Q = tkinter.Button(top, text="Quit", command=top.destroy)
        B.pack()
        B2.pack()
        Q.pack()
        top.mainloop()



if __name__ == '__main__':

    try:
        node = Manipulate_Arm()
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')