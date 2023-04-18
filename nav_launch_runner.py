import roslaunch
import rospy
from std_msgs.msg import Int8


class Launcher():
    def __init__(self):
        # rospy.init_node('navigation', anonymous=True)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        cli_args = ['/home/hello-robot/catkin_ws/src/stretch_ros/stretch_navigation/launch/navigation.launch','map_yaml:=/home/hello-robot/stretch_user/maps/yessir.yaml']
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        self.MM_publisher = rospy.Subscriber('/MM_flag', Int8, self.flag_callback)

        self.nav_launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        
    def flag_callback(self, msg):
        flag = msg.data
        if flag == 1:
            self.killnav()

    def launch_nav(self):
            self.nav_launch.start()
            rospy.loginfo("started")

    def killnav(self):
            rospy.loginfo("shutting_down")
            self.nav_launch.shutdown()
            rospy.signal_shutdown("Completed")



# roslaunch stretch_navigation navigation.launch map_yaml:=${HELLO_FLEET_PATH}/maps/finalmidmap.yaml

if __name__ == '__main__':
    rospy.init_node('Launcher')
    launcher = Launcher()
    launcher.launch_nav()
    rospy.spin()
