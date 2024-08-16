#!/usr/bin/env python3

import glob
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from skimage import io

class ImageLoad(object):
    def __init__(self):
        rospy.init_node('image_publisher')
        self.image_pub = rospy.Publisher('/lane_1/full_image', Image, queue_size = 1)
        
        self.rate = rospy.Rate(1)
        self.bridge = CvBridge()
        self.imgs = sorted(glob.glob("resources/testing_images/*.jpg"))
        self.index = 0  

    def image_loop(self): 
        while not rospy.is_shutdown():
            try:
                if self.index >= len(self.imgs):
                    self.index = 0  
                img = io.imread(self.imgs[self.index])

                img_msg = self.bridge.cv2_to_imgmsg(img, encoding = 'rgb8')

                self.image_pub.publish(img_msg)
                rospy.loginfo(f"Published image {self.imgs[self.index]}")
                rospy.sleep(5)

                self.index += 1

            except rospy.ROSInterruptException:
                rospy.logerr("ROS Interrupt Exception! Just ignore the exception!")

if __name__ == '__main__':
    node = ImageLoad()
    node.image_loop()
    rospy.spin()