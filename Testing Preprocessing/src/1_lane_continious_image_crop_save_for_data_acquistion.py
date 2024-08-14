import cv2
import os

''' CREATING FOLDERS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> '''

# Base folder path for saving cropped images
base_folder = 'single_lane/images'

# Create the base folders if they don't exist
lane_folder = f'lane_{1}'
for position in ['P1', 'P2', 'P3']:
    path = os.path.join(base_folder, lane_folder, position)
    if not os.path.exists(path):
        os.makedirs(path)

# Variables to track the onions in each position
lane = 1
onion_current = [None, None, None]  # Represents P1, P2, P3
orientation_current = ['A', 'B', 'C']

''' SAVE & CROP FUNCTION >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> '''


# Initialize onion count
onion_count = 0

def save_cropped_images(cropped_image, top_left, bottom_right, position, onion_number, orientation):
    """Crops the image at the given position and saves it."""
    cropped_onion = cropped_image[top_left[1]:bottom_right[1], top_left[0]:bottom_right[0]]
    lane_folder = f'lane_{lane}'
    position_folder = f'P{position}'
    save_path = os.path.join(base_folder, lane_folder, position_folder, f'lane_{lane}_{onion_number}{orientation}.jpg')
    cv2.imwrite(save_path, cropped_onion)


''' PROCESS EACH IMAGE >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> '''


# Example function to process each new image
def process_image(image_path):
    global onion_current, onion_count

    # Load the new image
    stitched_image = cv2.imread(image_path)

    # Removing the marginal area
    x, y = 30, 30
    x1, y1 = 702, 702
    cropped_image = stitched_image[y:y1, x:x1]

    # Define the positions for this lane
    positions = [
        ((85, 30), (180, 130)),  # P1
        ((45, 290), (145, 410)),  # P2
        ((60, 500), (190, 625)),  # P3
    ]

    # Update the onions and save the images
    new_onion_number = onion_count + 1
    for i in reversed(range(3)):
        if i == 0:
            # New onion comes into P1
            onion_current[i] = new_onion_number
            onion_count += 1
        else:
            # Shift the onion from P(i-1) to P(i)
            onion_current[i] = onion_current[i - 1]

        # Save the cropped image with the appropriate name
        if onion_current[i] is not None:
            save_cropped_images(cropped_image, *positions[i], i+1, onion_current[i], orientation_current[i])


''' IMAGES PATH >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> '''


# Example of processing multiple consecutive images
image_paths = [
    '/Users/user/Documents/ROS_new_potato/Testing Preprocessing/padded_image.jpg',
    '/Users/user/Documents/ROS_new_potato/Testing Preprocessing/padded_image.jpg',
    '/Users/user/Documents/ROS_new_potato/Testing Preprocessing/padded_image.jpg',
    '/Users/user/Documents/ROS_new_potato/Testing Preprocessing/padded_image.jpg'
    # Add more image paths as needed
]

for image_path in image_paths:
    process_image(image_path)

print("Cropped images saved successfully.")


