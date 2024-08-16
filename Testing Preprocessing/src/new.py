import os
import cv2
import numpy as np

# Function to rotate the image
def rotate_image(image, angle):
    if image is None:
        return None
    height, width = image.shape[:2]
    center = (width // 2, height // 2)
    rotation_matrix = cv2.getRotationMatrix2D(center, angle, 1)
    rotated_image = cv2.warpAffine(image, rotation_matrix, (width, height))
    return rotated_image

# Function to create the canvas without padding
def create_canvas():
    # Create a 672x672 image (no padding)
    canvas = np.zeros((672, 672, 3), dtype=np.uint8)
    return canvas

# Function to place the images on the canvas
def place_images_on_canvas(canvas, images):
    # Images is a list of 9 images to be placed in a 3x3 grid
    for i in range(3):
        for j in range(3):
            x_start = j * 224
            y_start = i * 224
            if images[i * 3 + j] is not None:
                canvas[y_start:y_start + 224, x_start:x_start + 224] = images[i * 3 + j]
            else:
                # Leave the block white if no image is provided
                canvas[y_start:y_start + 224, x_start:x_start + 224] = (255, 255, 255)
    return canvas

# Function to process the images and generate 10 outputs
def process_images(folder_path):
    # List all image files in the folder
    image_files = [f for f in os.listdir(folder_path) if f.endswith(('.jpg', '.jpeg', '.png'))]
    
    if len(image_files) < 9:
        print("Please provide at least 9 images.")
        return

    # Load the images
    images = [cv2.imread(os.path.join(folder_path, f)) for f in image_files]
    
    # Resize images to 224x224
    images = [cv2.resize(img, (224, 224)) for img in images]

    # Initialize the lanes with None (white blocks)
    lanes = [None] * 9
    used_indices = set()  # Track used image indices
    output_count = 1

    # Run the process for 10 output images
    for step in range(10):
        # Create a new canvas
        canvas = create_canvas()

        # Prepare the lanes
        if output_count == 1:
            # First step, fill row 1
            for i in range(3):
                lanes[i] = rotate_image(images[i], angle=45 * i)  # Fill row 1 with initial images
                used_indices.add(i)
        else:
            # Subsequent steps
            for i in range(3):
                lanes[i + 6] = None  # Remove row 3 images
                lanes[i + 6] = rotate_image(lanes[i + 3], angle=45)  # Move row 2 images to row 3 with rotation
                lanes[i + 3] = rotate_image(lanes[i], angle=45)  # Move row 1 images to row 2 with rotation
                
                # Find a new unused image for row 1 
                new_index = None
                for j in range(len(images)):
                    if j not in used_indices:
                        new_index = j
                        break
                
                if new_index is not None:
                    lanes[i] = rotate_image(images[new_index], angle=45 * (i + output_count))  # Fill row 1 with new images
                    used_indices.add(new_index)

        # Place images on canvas and save the image
        canvas = place_images_on_canvas(canvas, lanes)
        cv2.imwrite(os.path.join(folder_path, f'output_step_{output_count}.jpg'), canvas)
        output_count += 1

        # Reset used_indices if all images have been used
        if len(used_indices) >= len(images):
            used_indices.clear()

if __name__ == "__main__":
    folder_path = "/Users/user/Documents/ROS_new_potato/Testing Preprocessing/resources/images/"
    process_images(folder_path)