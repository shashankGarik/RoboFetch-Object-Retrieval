#!/usr/bin/env python3

# Import modules
import math
import rospy
import sys

from sensor_msgs.msg import JointState
from std_msgs.msg import Int32,Int8
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import hello_helpers.hello_misc as hm
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from gtts import gTTS
from playsound import playsound


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

        # Initialize the sound direction
        self.sound_direction = 0
        self.rad_per_deg = math.pi/180.0
        # Initialize subscribers
        self.speech_to_text_sub  = rospy.Subscriber("/speech_to_text",  SpeechRecognitionCandidates, self.callback_speech)
        self.sound_direction_sub = rospy.Subscriber("/sound_direction", Int32, self.callback_direction)

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
        print('                                           ')
        print('------------ VOICE TELEOP MENU ------------')
        print('                                           ')
        print('               VOICE COMMANDS              ')
        print(' "help": BASE FORWARD                   ')
        print(' "quit": QUIT AND CLOSE NODE            ')
        print('                                           ')
        print('-------------------------------------------')

    def get_command(self):
        """
        A function that defines the teleoperation command based on the voice command.
        :param self: The self reference.

        :returns command: A dictionary type that contains the type of base motion.
        """
        command = 0
        # Move base forward command
        if self.voice_command == 'help':
            print(self.voice_command)
            thanks = gTTS("Please hold on! I am going to get the medicine")
            thanks.save("hello.mp3")
            playsound("hello.mp3")
            command = 1

        if self.voice_command == 'quit':
            print(self.voice_command)
            # Sends a signal to rospy to shutdown the ROS interfaces
            rospy.signal_shutdown("done")

            # Exit the Python interpreter
            sys.exit(0)

        # Reset voice command to None
        self.voice_command = None

        # return the updated command
        return command


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
        self.button_publisher = rospy.Publisher('/voice', Int8, queue_size=10)
        self.listener = rospy.Subscriber('/button', Int8, self.button_callback)

    def button_callback(self, msg):
        self.do_state = msg.data
        if self.do_state == 1:
            rospy.signal_shutdown("Process completed")

    def main(self):
        """
        The main function that instantiates the HelloNode class, initializes the subscriber,
        and call other methods in both the VoiceTeleopNode and GetVoiceCommands classes.
        :param self: The self reference.
        """
        hm.HelloNode.main(self, 'voice_teleop', 'voice_teleop', wait_for_first_pointcloud=False)
        rate = rospy.Rate(self.rate)
        self.speech.print_commands()

        while not rospy.is_shutdown():
            # Get voice command
            command = self.speech.get_command()
            if command == 1:
                self.button_publisher.publish(1)
                rospy.signal_shutdown("Process completed")

            rate.sleep()

if __name__ == '__main__':
    try:
        node = VoiceTeleopNode()
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')
