import cv2
import os

''' CREATING FOLDERS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> '''

base_folder = 'resources/RGB_three_lane/images'

lanes = 3
for lane in range(1, lanes + 1):
    lane_folder = f'lane_{lane}'
    for orientation in ['A', 'B', 'C']:
        path = os.path.join(base_folder, lane_folder, orientation)
        if not os.path.exists(path):
            os.makedirs(path)


onion_current = [[None, None, None] for _ in range(lanes)]  
orientation_current = ['A', 'B', 'C']

# Initialize onion count for each lane
onion_count = [0] * lanes


''' SAVE & CROP FUNCTION >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> '''

def save_cropped_images(cropped_image, top_left, bottom_right, lane, orientation, onion_number):
    """Crops the image at the given position and saves it."""
    cropped_onion = cropped_image[top_left[1]:bottom_right[1], top_left[0]:bottom_right[0]]
    lane_folder = f'lane_{lane}'
    orientation_folder = orientation
    save_path = os.path.join(base_folder, lane_folder, orientation_folder, f'lane_{lane}_{onion_number}{orientation}.jpg')
    cv2.imwrite(save_path, cropped_onion)

''' PROCESS EACH IMAGE >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> '''

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


def process_image(image_path):
    global onion_current, onion_count
    stitched_image = cv2.imread(image_path)

    shape = stitched_image.shape

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

    # Process each lane
    for lane in range(lanes):
        new_onion_number = onion_count[lane] + 1
        for i in reversed(range(3)):
            if i == 0:
                onion_current[lane][i] = new_onion_number
                onion_count[lane] += 1
            else:
                onion_current[lane][i] = onion_current[lane][i - 1]

            if onion_current[lane][i] is not None:
                save_cropped_images(cropped_image, areas_of_interest[lane * 3 + i][0], areas_of_interest[lane * 3 + i][1],
                                    lane + 1, orientation_current[i], onion_current[lane][i])


image_paths = [
    '/Users/user/Documents/ROS_new_potato/Testing Preprocessing/resources/padded_image.jpg',
    '/Users/user/Documents/ROS_new_potato/Testing Preprocessing/resources/padded_image.jpg',
    '/Users/user/Documents/ROS_new_potato/Testing Preprocessing/resources/padded_image.jpg',
    '/Users/user/Documents/ROS_new_potato/Testing Preprocessing/resources/padded_image.jpg',
]

# image_paths = [
#     '/Users/user/Documents/ROS_new_potato/Testing Preprocessing/padded_image.jpg',
#     '/Users/user/Documents/ROS_new_potato/Testing Preprocessing/padded_image.jpg',
#     '/Users/user/Documents/ROS_new_potato/Testing Preprocessing/padded_image.jpg',
#     '/Users/user/Documents/ROS_new_potato/Testing Preprocessing/padded_image.jpg'
# ]

for image_path in image_paths:
    process_image(image_path)

print("Cropped images saved")