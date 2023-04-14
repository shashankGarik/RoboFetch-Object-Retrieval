import rospy
import stretch_body.xbox_controller as xc
from std_msgs.msg import Int8


class buttonPress(object):
    def __init__(self):

        self.xbox_controller = xc.XboxController()
        self.xbox_controller.start()

        self.button_state = 0
        self.button_publisher = rospy.Publisher('/button', Int8, queue_size=10)
        self.voice_command = rospy.Subscriber('/voice', Int8, self.voice_command_callback)
    
    def voice_command_callback(self, msg):
        self.do_state = msg.data
        if self.do_state == 1:
            rospy.signal_shutdown("Process completed")


    def controller_state(self): 
        controller_state = self.xbox_controller.get_state()

        right_button = controller_state['right_button_pressed']

        if right_button == True:
                print('Button pressed')
                self.button_state = 1
                print(self.button_state)       

        self.button_publisher.publish(self.button_state)
        
        if self.button_state ==  1:
            rospy.signal_shutdown("Process completed")



if __name__ == '__main__':

    rospy.init_node('button_state')
    stetch_button = buttonPress()
    print('Press Right Button')
    while not rospy.is_shutdown():
        stetch_button.controller_state()
