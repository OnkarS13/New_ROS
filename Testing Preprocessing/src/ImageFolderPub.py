#!/usr/bin/env python3

import glob
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from skimage import io
import os

class ImageLoad(object):
    def __init__(self):
        rospy.init_node('image_publisher')
        # Publishers for three different topics corresponding to positions P1, P2, P3
        self.pub_p1 = rospy.Publisher('/lane_1/position_p1/image', Image, queue_size=1)
        self.pub_p2 = rospy.Publisher('/lane_1/position_p2/image', Image, queue_size=1)
        self.pub_p3 = rospy.Publisher('/lane_1/position_p3/image', Image, queue_size=1)
        
        self.rate = rospy.Rate(1)
        self.bridge = CvBridge()

        # Load images from folders A, B, C
        self.imgs_a = sorted(glob.glob("/home/agrograde/agrograde_ws/src/multilane_sorter/assets/images/lane_1/A/*.jpg"))
        self.imgs_b = sorted(glob.glob("/home/agrograde/agrograde_ws/src/multilane_sorter/assets/images/lane_1/B/*.jpg"))
        self.imgs_c = sorted(glob.glob("/home/agrograde/agrograde_ws/src/multilane_sorter/assets/images/lane_1/C/*.jpg"))

        self.index = 0  # To track the current image index

    def image_loop(self): 
        while not rospy.is_shutdown():
            try:
                # First publish: only from folder A (P1)
                if self.index < len(self.imgs_a):
                    img_p1 = io.imread(self.imgs_a[self.index])
                    img_p1_msg = self.bridge.cv2_to_imgmsg(img_p1, encoding='rgb8')
                    self.pub_p1.publish(img_p1_msg)
                    rospy.loginfo("Published image from folder A (P1)")

                rospy.sleep(5)  # Wait for 5 seconds

                # Second publish: from folder B (P2) and A (P1)
                if self.index < len(self.imgs_b):
                    img_p2 = io.imread(self.imgs_b[self.index])
                    img_p2_msg = self.bridge.cv2_to_imgmsg(img_p2, encoding='rgb8')
                    self.pub_p2.publish(img_p2_msg)
                    rospy.loginfo("Published image from folder B (P2)")

                if self.index + 1 < len(self.imgs_a):
                    img_p1 = io.imread(self.imgs_a[self.index + 1])
                    img_p1_msg = self.bridge.cv2_to_imgmsg(img_p1, encoding='rgb8')
                    self.pub_p1.publish(img_p1_msg)
                    rospy.loginfo("Published image from folder A (P1)")

                rospy.sleep(5)  # Wait for 5 seconds

                # Third publish: from folder C (P3), B (P2), and A (P1)
                if self.index < len(self.imgs_c):
                    img_p3 = io.imread(self.imgs_c[self.index])
                    img_p3_msg = self.bridge.cv2_to_imgmsg(img_p3, encoding='rgb8')
                    self.pub_p3.publish(img_p3_msg)
                    rospy.loginfo("Published image from folder C (P3)")

                if self.index + 1 < len(self.imgs_b):
                    img_p2 = io.imread(self.imgs_b[self.index + 1])
                    img_p2_msg = self.bridge.cv2_to_imgmsg(img_p2, encoding='rgb8')
                    self.pub_p2.publish(img_p2_msg)
                    rospy.loginfo("Published image from folder B (P2)")

                if self.index + 2 < len(self.imgs_a):
                    img_p1 = io.imread(self.imgs_a[self.index + 2])
                    img_p1_msg = self.bridge.cv2_to_imgmsg(img_p1, encoding='rgb8')
                    self.pub_p1.publish(img_p1_msg)
                    rospy.loginfo("Published image from folder A (P1)")

                rospy.sleep(5)  # Wait for 5 seconds

                # Increment index to move to the next set of images
                self.index += 1

                # Reset index if we've reached the end of the image lists
                if self.index >= len(self.imgs_a) or self.index >= len(self.imgs_b) or self.index >= len(self.imgs_c):
                    self.index = 0

            except rospy.ROSInterruptException:
                rospy.logerr("ROS Interrupt Exception! Just ignore the exception!")

if __name__ == '__main__':
    node = ImageLoad()
    node.image_loop()
    rospy.spin()