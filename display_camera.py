import rospy

import cv2
import numpy as np
from sensor_msgs.msg import Imu

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo

class DisplayCamera:
    def __init__(self):
        # self.image = np.zeros((1920,1080))
        self.bridge = CvBridge()
        
    def image_callback(self, img):
        # self.image = img

        # print(np.shape(self.image))

        image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        rotated_img = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)

        cv2.imshow('frame', rotated_img)
        cv2.waitKey(1)
       

            
    def main(self):
        rospy.init_node('DisplayCamera')
        self.node_name = rospy.get_name()
        rospy.loginfo("{0} started".format(self.node_name))

        self.topic_name = '/camera/color/image_raw/compressed'
        self.image_subscriber = rospy.Subscriber(self.topic_name, Image, self.image_callback)
        
        # self.corrected_accel_pub = rospy.Publisher('/camera/accel/sample_corrected', Imu, queue_size=1)


if __name__ == '__main__':
    node = DisplayCamera()
    node.main()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('interrupt received, so shutting down')



# #!/usr/bin/env python3

# import sys
# import rospy
# from sensor_msgs.msg import Imu

# class D435iAccelCorrectionNode:
#     def __init__(self):
#         self.num_samples_to_skip = 4
#         self.sample_count = 0
        
#     def accel_callback(self, accel):
#         self.accel = accel
#         self.sample_count += 1
#         if (self.sample_count % self.num_samples_to_skip) == 0: 
#             # This can avoid issues with the D435i's time stamps being too
#             # far ahead for TF.
#             self.accel.header.stamp = rospy.Time.now()
#             x = self.accel.linear_acceleration.x
#             y = self.accel.linear_acceleration.y
#             z = self.accel.linear_acceleration.z

#             print(x)

#             self.accel.linear_acceleration.x = x
#             self.accel.linear_acceleration.y = y
#             self.accel.linear_acceleration.z = z

#             self.corrected_accel_pub.publish(self.accel)

            
#     def main(self):
#         rospy.init_node('D435iAccelCorrectionNode')
#         self.node_name = rospy.get_name()
#         rospy.loginfo("{0} started".format(self.node_name))

#         self.topic_name = '/camera/accel/sample'
#         self.accel_subscriber = rospy.Subscriber(self.topic_name, Imu, self.accel_callback)
        
#         self.corrected_accel_pub = rospy.Publisher('/camera/accel/sample_corrected', Imu, queue_size=1)


# if __name__ == '__main__':
#     node = D435iAccelCorrectionNode()
#     node.main()
#     try:
#         rospy.spin()
#     except KeyboardInterrupt:
#         print('interrupt received, so shutting down')