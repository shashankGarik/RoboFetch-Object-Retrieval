#!/usr/bin/env python3

import rospy
import os
from speech_recognition_msgs.msg import SpeechRecognitionCandidates

class SpeechText:
    """
    A class that saves the interpreted speech from the ReSpeaker Microphone Array to a text file.
    """
    def __init__(self):
        """
        Initialize subscriber and directory to save speech to text file.
        """
        self.sub = rospy.Subscriber("speech_to_text", SpeechRecognitionCandidates, self.callback)
        self.save_path = '~/catkin_ws/src/robo_fetch/src'
        rospy.loginfo("Listening to speech.")

    def callback(self,msg):
        """
        A callback function that receives the speech transcript and appends the
        transcript to a text file.
        :param self: The self reference.
        :param msg: The SpeechRecognitionCandidates message type.
        """
        transcript = ' '.join(map(str,msg.transcript))
        file_name = 'speech.txt'
        completeName = os.path.join(self.save_path, file_name)
        print(transcript)
        with open(completeName, "a+") as file_object:
            file_object.write("\n")
            file_object.write(transcript)

if __name__ == '__main__':
    rospy.init_node('speech_text')
    SpeechText()
    rospy.spin()