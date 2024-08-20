#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from multilane_sorter.msg import inference
import cv2
import numpy as np
import onnxruntime

class AI_Node:
    def __init__(self):
        rospy.init_node('ai_node')
        self.bridge = CvBridge()

        # Initialize the Segmentation model
        model_path = "//home/agrograde/agrograde_ws/src/multilane_sorter/ai_models/30th_july_2024/four_class_onnx_model.onnx"
        self.model = Segmentation_model(model_path)

        # Initialize lists to store images for each position
        self.image_queue = [[], [], [], []]  # 4 positions for 4 orientations

        # Step tracker
        self.step = 1

        # Subscriber for cropped image topic
        rospy.Subscriber('/lane_1/cropped_image', Image, self.image_callback)

        # Publisher for AI inference results
        self.inference_pub = rospy.Publisher('ai_inference_channel', inference, queue_size=10)

        # Initialize result lists for different positions
        self.p_new = []
        self.p_next = []
        self.p_current = []
        self.p_final = []

    def image_callback(self, msg):
        # Process each incoming image message
        position = self.step - 1  # Get current position (0 = P1, 1 = P2, 2 = P3, 3 = P4)
        self.image_queue[position].append(msg)
        self.process_images()

    def process_images(self):
        # Process images based on the current step
        if self.step == 1 and self.image_queue[0]:
            # Process the first image for P1
            img_p1 = self.image_queue[0].pop(0)
            self.process_image(img_p1, 'P1')
            self.step += 1

        elif self.step == 2 and self.image_queue[0] and self.image_queue[1]:
            # Process images for P2 and P1
            img_p2 = self.image_queue[1].pop(0)
            img_p1 = self.image_queue[0].pop(0)
            self.process_image(img_p2, 'P2')
            self.process_image(img_p1, 'P1')
            self.step += 1

        elif self.step == 3 and self.image_queue[0] and self.image_queue[1] and self.image_queue[2]:
            # Process images for P3, P2, P1
            img_p3 = self.image_queue[2].pop(0)
            img_p2 = self.image_queue[1].pop(0)
            img_p1 = self.image_queue[0].pop(0)
            self.process_image(img_p3, 'P3')
            self.process_image(img_p2, 'P2')
            self.process_image(img_p1, 'P1')
            self.step += 1

        elif self.step == 4 and self.image_queue[0] and self.image_queue[1] and self.image_queue[2] and self.image_queue[3]:
            # Process images for P4, P3, P2, P1
            img_p4 = self.image_queue[3].pop(0)
            img_p3 = self.image_queue[2].pop(0)
            img_p2 = self.image_queue[1].pop(0)
            img_p1 = self.image_queue[0].pop(0)
            self.process_image(img_p4, 'P4')
            self.process_image(img_p3, 'P3')
            self.process_image(img_p2, 'P2')
            self.process_image(img_p1, 'P1')
            
            self.update_lists()

    def process_image(self, msg, position):

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Run the actual inference model on the image
        result, size = self.model.getPrediction_values(cv_image)

        # Update lists based on the position
        if position == 'P1':
            if self.step == 1 or len(self.p_new) == 0:
                self.p_new.append(result)
            else:
                self.p_new[0] = result
        elif position == 'P2':
            if len(self.p_next) == 0:
                self.p_next.append(result)
            else:
                self.p_next[1] = result
        elif position == 'P3':
            if len(self.p_current) == 0:
                self.p_current.append(result)
            else:
                self.p_current[2] = result
        elif position == 'P4':
            if len(self.p_final) == 0:
                self.p_final.append(result)
            else:
                self.p_final[3] = result

    def message(self, array):

        array = [round(item, 2) for item in array]
        inf_msg = inference()
        inf_msg.sprout = array[0]
        inf_msg.peeled = array[1]
        inf_msg.rotten = array[2]
        inf_msg.blacksmut = array[3]
        inf_msg.size = array[4]

        self.inference_pub.publish(inf_msg)
        rospy.loginfo(f"Published results: {array}")

    def update_lists(self):

        if len(self.p_final) == 4:
            self.message(self.p_final)
            self.p_final.clear()

        if len(self.p_current) > 0:
            self.p_final = list(self.p_current)
            self.p_current.clear()

        if len(self.p_next) > 0:
            self.p_current = list(self.p_next)
            self.p_next.clear()

        if len(self.p_new) > 0:
            self.p_next = list(self.p_new)
            self.p_new.clear()

class Segmentation_model():
    def __init__(self, model_path):
        self.model_path = model_path
        self.session = onnxruntime.InferenceSession(self.model_path)
        self.input_name = self.session.get_inputs()[0].name
        self.output_names = [output.name for output in self.session.get_outputs()]

    def predict(self, image):
        result = self.session.run(self.output_names, {self.input_name: image})
        return result

    def getPercentArea(self, full_mask, region_mask):
        total_area = np.dot(full_mask.flatten(), np.ones_like(full_mask.flatten()))
        region_area = np.dot(region_mask.flatten(), np.ones_like(region_mask.flatten()))
        area_percentage = (region_area / total_area) * 100
        return area_percentage

    def getPrediction_values(self, img_path):
        h, w = 224, 224
        
        im = cv2.resize(img_path, (h, w))
        I = im.astype(np.float32)
        I = I.reshape([1, h, w, 3])
        
        preds = self.predict(I)
        
        sp = np.argmax(preds[0], axis=3).reshape([h, w])
        pl = np.argmax(preds[1], axis=3).reshape([h, w])
        ro = np.argmax(preds[2], axis=3).reshape([h, w])
        bs = np.argmax(preds[3], axis=3).reshape([h, w])
        bg = np.argmax(preds[4], axis=3).reshape([h, w])

        gray_img = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        ret, binary = cv2.threshold(gray_img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        contours, hierarchy = cv2.findContours(binary, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
        max_area = 0
        biggest_contour = None
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                biggest_contour = contour
        
        ellipse = cv2.fitEllipse(biggest_contour)
        size = max(ellipse[1]) * 1.47489

        sprout_area = self.getPercentArea(bg, sp)
        peeled_area = self.getPercentArea(bg, pl)
        rotten_area = self.getPercentArea(bg, ro)
        black_smut_area = self.getPercentArea(bg, bs)
        background_area = self.getPercentArea(bg, bg)
        total_area = background_area

        final_percentage_features = [
            (sprout_area * 100) / total_area,
            (peeled_area * 100) / total_area,
            (rotten_area * 100) / total_area,
            (black_smut_area * 100) / total_area,
            size
        ]

        return final_percentage_features, size

if __name__ == '__main__':
    node = AI_Node()
    rospy.spin()