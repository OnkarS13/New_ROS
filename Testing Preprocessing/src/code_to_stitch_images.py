import cv2
import numpy as np

image_paths = [
    '/Users/user/Documents/ROS_new_potato/Testing Preprocessing/resources/images/1_.jpg',
    '/Users/user/Documents/ROS_new_potato/Testing Preprocessing/resources/images/2_.jpg',
    '/Users/user/Documents/ROS_new_potato/Testing Preprocessing/resources/images/3_.jpg',
    '/Users/user/Documents/ROS_new_potato/Testing Preprocessing/resources/images/4_.jpg',
    '/Users/user/Documents/ROS_new_potato/Testing Preprocessing/resources/images/5_.jpg',
    '/Users/user/Documents/ROS_new_potato/Testing Preprocessing/resources/images/6_.jpg',
    '/Users/user/Documents/ROS_new_potato/Testing Preprocessing/resources/images/7_.jpg',
    '/Users/user/Documents/ROS_new_potato/Testing Preprocessing/resources/images/8_.jpg',
    '/Users/user/Documents/ROS_new_potato/Testing Preprocessing/resources/images/9_.jpg',
    '/Users/user/Documents/ROS_new_potato/Testing Preprocessing/resources/images/10_.jpg', 
    '/Users/user/Documents/ROS_new_potato/Testing Preprocessing/resources/images/11_.jpg', 
    '/Users/user/Documents/ROS_new_potato/Testing Preprocessing/resources/images/12_.jpg'
]

# Load all images
images = [cv2.imread(image_path) for image_path in image_paths]

# Check if any image failed to load
for i, img in enumerate(images):
    if img is None:
        print(f"Failed to load image at {image_paths[i]}")
        images[i] = np.zeros((224, 224, 3), dtype=np.uint8)  # Replace with a black image to avoid errors

# Resize all images to the same size
image_size = (224, 224)
images = [cv2.resize(img, image_size) for img in images]

# Combine images row by row
row1 = np.hstack(images[0:3])  # First row
row2 = np.hstack(images[3:6])  # Second row
row3 = np.hstack(images[6:9])  # Third row
row4 = np.hstack(images[9:12])  # Fourth row

# Combine all rows vertically to create a 4x3 matrix
stitched_image = np.vstack([row1, row2, row3, row4])

# Save the stitched image into the resources folder
output_path = '/Users/user/Documents/ROS_new_potato/Testing Preprocessing/resources/stitched_image.jpg'
cv2.imwrite(output_path, stitched_image)

# Optionally display the stitched image (ensure your environment supports GUI)
cv2.imshow('Stitched Image', stitched_image)
cv2.waitKey(0)
cv2.destroyAllWindows()