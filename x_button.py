import rospy
import stretch_body.xbox_controller as xc
from std_msgs.msg import Int8


class buttonPress(object):
    def __init__(self):

        self.xbox_controller = xc.XboxController()
        self.xbox_controller.start()

        self.button_state = 0
        self.button_publisher = rospy.Publisher('/button', Int8, queue_size=10)

    def controller_state(self):
        controller_state = self.xbox_controller.get_state()

        # print(controller_state)

        right_button = controller_state['right_button_pressed']

        if right_button == False and self.button_state == 0:
            print('Press Right Button')

        if right_button == True:
                print('Getting here')
                self.button_state = 1
            

        if right_button == False and self.button_state == 1:
            print('Button Pressed, Executing Grasp')

        self.button_publisher.publish(self.button_state)
        print(self.button_state)



if __name__ == '__main__':
    rospy.init_node('button_state')
    stetch_button = buttonPress()
    while not rospy.is_shutdown():
        stetch_button.controller_state()
    # rospy.spin()