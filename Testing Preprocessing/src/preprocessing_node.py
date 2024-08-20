#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class PreProcessingNode:
    def __init__(self):
        rospy.init_node('preprocessing_node')
        self.bridge = CvBridge()


        # Adjusting parameter (width, height) --> (x, y)
        # Lane 1
        l1_p1_x1, l1_p1_y1 = 50, 45
        l1_p1_x2, l1_p1_y2 = 170, 160

        l1_p2_x1, l1_p2_y1 = 45, 245
        l1_p2_x2, l1_p2_y2 = 155, 370

        l1_p3_x1, l1_p3_y1 = 60, 460
        l1_p3_x2, l1_p3_y2 = 180, 580

        l1_p4_x1, l1_p4_y1 = 50, 700
        l1_p4_x2, l1_p4_y2 = 225, 845

        # Define the areas of interest for Lane 1
        self.areas_of_interest = [
            # A
            ((l1_p1_x1, l1_p1_y1), 
            (l1_p1_x2, l1_p1_y2)),  
            # B
            ((l1_p2_x1, l1_p2_y1), 
            (l1_p2_x2, l1_p2_y2)),  
            # C
            ((l1_p3_x1, l1_p3_y1), 
            (l1_p3_x2, l1_p3_y2)),
            # D
            ((l1_p4_x1, l1_p4_y1), 
            (l1_p4_x2, l1_p4_y2)),
        ]

        # Initialize variables to track onions and their orientations
        self.onion_current = [None, None, None, None]  # Tracking onions in Lane 1 for orientations A, B, C, D
        self.onion_count = 0  # Counter for onions in Lane 1
        self.orientation_current = ['A', 'B', 'C', 'D']  # Orientations

        # Single publisher to publish cropped images
        self.image_pub = rospy.Publisher('/lane_1/cropped_image', Image, queue_size=10)

        # Subscribe to the full image topic
        rospy.Subscriber('/lane_1/full_image', Image, self.image_callback)

    def image_callback(self, msg):
        # Convert the ROS Image message to an OpenCV image
        stitched_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Process the image as before
        self.process_and_publish_images(stitched_image)

    def process_and_publish_images(self, stitched_image):
        # Get the shape of the image
        shape = stitched_image.shape

        # Define the margins to remove
        margin_to_remove_left = 30
        margin_to_remove_right = 30
        margin_to_remove_top = 30
        margin_to_remove_bottom = 30

        y_height = shape[0]
        x_width = shape[1]

        # Remove the marginal area
        x, y = margin_to_remove_left, margin_to_remove_top
        x1, y1 = (x_width - margin_to_remove_right), (y_height - margin_to_remove_bottom)
        cropped_image = stitched_image[y:y1, x:x1]

        # Process Lane 1 only
        new_onion_number = self.onion_count + 1
        for i in reversed(range(4)):  # Handle 4 orientations (D, C, B, A)
            if i == 0:
                self.onion_current[i] = new_onion_number
                self.onion_count += 1
            else:
                self.onion_current[i] = self.onion_current[i - 1]

            if self.onion_current[i] is not None:
                top_left, bottom_right = self.areas_of_interest[i]
                cropped_onion = cropped_image[top_left[1]:bottom_right[1], top_left[0]:bottom_right[0]]

                # Convert the cropped image to ROS Image message and publish it
                cropped_onion_msg = self.bridge.cv2_to_imgmsg(cropped_onion, encoding="bgr8")
                self.image_pub.publish(cropped_onion_msg)

                rospy.loginfo(f"Published cropped onion at orientation {self.orientation_current[i]}: Lane 1, Onion {self.onion_current[i]}")
                # rospy.sleep(5)  # Sleep between publishing each image for better sequencing

if __name__ == '__main__':
    node = PreProcessingNode()
    rospy.spin()