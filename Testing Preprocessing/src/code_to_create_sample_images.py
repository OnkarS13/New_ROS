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

# Function to create the 672x896 canvas (3 lanes, 4 images each, no padding)
def create_canvas():
    canvas = np.zeros((896, 672, 3), dtype=np.uint8)
    return canvas

# Function to create a padded canvas with 30-pixel green padding
def add_padding(image):
    padded_canvas = np.zeros((956, 732, 3), dtype=np.uint8)  # Create a larger canvas (896 + 2*30, 672 + 2*30)
    padded_canvas[:, :] = (0, 255, 0)  # Fill the canvas with green color

    # Place the original image in the center of the padded canvas
    padded_canvas[30:30 + 896, 30:30 + 672] = image
    return padded_canvas

# Function to place the images on the canvas
def place_images_on_canvas(canvas, images):
    # Images is a list of 12 images to be placed in a 4x3 grid (4 images per lane, 3 lanes)
    for i in range(4):  # 4 images per lane
        for j in range(3):  # 3 lanes
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
    
    if len(image_files) < 12:
        print("Please provide at least 12 images.")
        return

    # Load the images
    images = [cv2.imread(os.path.join(folder_path, f)) for f in image_files]
    
    # Resize images to 224x224
    images = [cv2.resize(img, (224, 224)) for img in images]

    # Initialize the lanes with None (white blocks)
    lanes = [None] * 12
    used_indices = set()  # Track used image indices
    output_count = 1

    # Run the process for 10 output images
    for step in range(10):
        # Create a new canvas
        canvas = create_canvas()

        # Prepare the lanes
        if output_count == 1:
            # First step, fill the first images in each lane
            for i in range(3):  # 3 lanes
                lanes[i] = rotate_image(images[i], angle=45 * i)  # Fill first position in each lane with initial images
                used_indices.add(i)
        else:
            # Subsequent steps
            for i in range(3):
                lanes[i + 9] = None  # Remove the last image in each lane (4th position)
                lanes[i + 9] = rotate_image(lanes[i + 6], angle=45)  # Move 3rd image to 4th position
                lanes[i + 6] = rotate_image(lanes[i + 3], angle=45)  # Move 2nd image to 3rd position
                lanes[i + 3] = rotate_image(lanes[i], angle=45)  # Move 1st image to 2nd position
                
                # Find a new unused image for the first position in each lane
                new_index = None
                for j in range(len(images)):
                    if j not in used_indices:
                        new_index = j
                        break
                
                if new_index is not None:
                    lanes[i] = rotate_image(images[new_index], angle=45 * (i + output_count))  # Fill first position with new images
                    used_indices.add(new_index)

        # Place images on canvas
        canvas = place_images_on_canvas(canvas, lanes)
        
        # Add green padding
        padded_canvas = add_padding(canvas)

        # Save the padded image
        folder_to_save = 'resources/testing_images'
        cv2.imwrite(os.path.join(folder_to_save, f'output_step_{output_count}.jpg'), padded_canvas)
        output_count += 1

        # Reset used_indices if all images have been used
        if len(used_indices) >= len(images):
            used_indices.clear()

if __name__ == "__main__":
    folder_path = "resources/images"
    process_images(folder_path)