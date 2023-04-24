import roslaunch
import rospy
from std_msgs.msg import Int8


class Launcher2():
    def __init__(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.aruco_publisher = rospy.Publisher('/start_aruco_flag', Int8, queue_size = 10)

        self.retrival_launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/hello-robot/catkin_ws/src/robo_fetch/launch/retrieval.launch"])
        

    def launch_retrival(self):
            rospy.sleep(5)
            self.retrival_launch.start()
            rospy.loginfo("started")
            rospy.sleep(10)
            self.aruco_publisher.publish(1)

    def kill_retrival(self):
            rospy.loginfo("shutting_down")
            self.retrival_launch.shutdown()
            rospy.signal_shutdown("Completed")


if __name__ == '__main__':
    rospy.init_node('Launcher2')
    launcher2 = Launcher2()
    launcher2.launch_retrival()
    rospy.spin()
