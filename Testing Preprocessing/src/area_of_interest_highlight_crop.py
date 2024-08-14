import cv2
import numpy as np
import os

# Load the stitched image
stitched_image = cv2.imread('/Users/user/Documents/ROS_new_potato/Testing Preprocessing/resources/padded_image.jpg')
cv2.imshow('cropped Image', stitched_image)
cv2.waitKey(0)

shape = stitched_image.shape
print(f"Shape of image before padding removal: {shape}")


''' REMOVING MARGINAL PART >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> '''

margin_to_remove_left = 30
margin_to_remove_right = 30
margin_to_remove_top = 30
margin_to_remove_bottom = 30

y_height = shape[0]
x_width = shape[1]

# Removing the marginal area
x, y = margin_to_remove_left, margin_to_remove_top
x1, y1 = (x_width - margin_to_remove_right), (y_height - margin_to_remove_bottom)

# shape[h, w, channels]
# shape[y_axis, x_axis] -->[height, width]
# image[height, width]

cropped_image = stitched_image[y:y1, x:x1]

# Shape of cropped image
cropped_image_shape = cropped_image.shape
print(f"Shape of image after padding removal: {cropped_image_shape}")


cv2.imshow('cropped Image', cropped_image)
output_path = '/Users/user/Documents/ROS_new_potato/Testing Preprocessing/resources/After_padding_removal_image.jpg'
cv2.imwrite(output_path, cropped_image)
cv2.waitKey(0)


''' ACCESSING EACH IMAGE INDEPENDLTY >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> '''

# Adjusting parameter (width, height) --> (x, y)
# Lane : 1
l1_p1_x1, l1_p1_y1 = 50, 45
l1_p1_x2, l1_p1_y2 = 170, 160

l1_p2_x1, l1_p2_y1 = 45, 245
l1_p2_x2, l1_p2_y2 = 155, 370

l1_p3_x1, l1_p3_y1 = 60, 460
l1_p3_x2, l1_p3_y2 = 180, 580

l1_p4_x1, l1_p4_y1 = 50, 700
l1_p4_x2, l1_p4_y2 = 225, 845

# Lane : 2
l2_p1_x1, l2_p1_y1 = 270, 10
l2_p1_x2, l2_p1_y2 = 400, 140

l2_p2_x1, l2_p2_y1 = 280, 230
l2_p2_x2, l2_p2_y2 = 450, 400

l2_p3_x1, l2_p3_y1 = 260, 450
l2_p3_x2, l2_p3_y2 = 420, 590

l2_p4_x1, l2_p4_y1 = 280, 670
l2_p4_x2, l2_p4_y2 = 400, 800

# Lane : 3
l3_p1_x1, l3_p1_y1 = 500, 20
l3_p1_x2, l3_p1_y2 = 660, 200

l3_p2_x1, l3_p2_y1 = 500, 230
l3_p2_x2, l3_p2_y2 = 630, 360

l3_p3_x1, l3_p3_y1 = 500, 500
l3_p3_x2, l3_p3_y2 = 620, 650

l3_p4_x1, l3_p4_y1 = 500, 680
l3_p4_x2, l3_p4_y2 = 660, 860

# Define the areas of interest with increased size
# area_of_interest = [((x1, y1), (x2, y2))]
areas_of_interest = [

    # Lane 1
    # P-1
    ((l1_p1_x1, l1_p1_y1), 
     (l1_p1_x2, l1_p1_y2)),  
    # P-2
    ((l1_p2_x1, l1_p2_y1), 
     (l1_p2_x2, l1_p2_y2)),  
    # P-3
    ((l1_p3_x1, l1_p3_y1), 
     (l1_p3_x2, l1_p3_y2)),
    # P-4
    ((l1_p4_x1, l1_p4_y1), 
     (l1_p4_x2, l1_p4_y2)),

    # Lane 2
    # P-1
    ((l2_p1_x1, l2_p1_y1), 
     (l2_p1_x2, l2_p1_y2)),  
    # P-2
    ((l2_p2_x1, l2_p2_y1), 
     (l2_p2_x2, l2_p2_y2)),  
    # P-3
    ((l2_p3_x1, l2_p3_y1), 
     (l2_p3_x2, l2_p3_y2)),
    # P-4
    ((l2_p4_x1, l2_p4_y1), 
     (l2_p4_x2, l2_p4_y2)),

    # Lane 3
    # P-1
    ((l3_p1_x1, l3_p1_y1), 
     (l3_p1_x2, l3_p1_y2)),  
    # P-2
    ((l3_p2_x1, l3_p2_y1), 
     (l3_p2_x2, l3_p2_y2)),  
    # P-3
    ((l3_p3_x1, l3_p3_y1), 
     (l3_p3_x2, l3_p3_y2)),
    # P-4
    ((l3_p4_x1, l3_p4_y1), 
     (l3_p4_x2, l3_p4_y2)),
]

''' HIGHLIGHTING EACH IMAGE >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> '''

# Highlight each area by drawing a rectangle
highlighted_image = cropped_image.copy()
for (x, y) in areas_of_interest:
    cv2.rectangle(highlighted_image, x, y, color = (0, 255, 0), thickness = 1)

# Display the image with highlighted areas
cv2.imshow('Highlighted Areas of Interest', highlighted_image)
output_path = '/Users/user/Documents/ROS_new_potato/Testing Preprocessing/resources/highlighted_image_increased.jpg'
cv2.imwrite(output_path, highlighted_image)  # Optional: Save the highlighted image
cv2.waitKey(0)
cv2.destroyAllWindows()

