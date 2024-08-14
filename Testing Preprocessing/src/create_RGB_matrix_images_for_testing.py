import numpy as np
import cv2

# Create an empty RGB image of shape (732, 732, 3)
image = np.zeros((732, 732, 3), dtype=np.uint8)

# Convert RGB to BGR (OpenCV uses BGR by default)
def rgb_to_bgr(rgb_color):
    return (rgb_color[2], rgb_color[1], rgb_color[0])

# Define the green color in RGB and convert it to BGR
green = rgb_to_bgr((0, 255, 0))

# Fill the borders with green color
image[:30, :] = green  # Top border
image[-30:, :] = green  # Bottom border
image[:, :30] = green  # Left border
image[:, -30:] = green  # Right border

# Define the coordinates for the 3x3 grid of 224x224 boxes
start_x = 30
start_y = 30
box_size = 224
gap = 0  # No gap between boxes

# Define the colors for each 224x224 box in RGB and convert them to BGR
colors = [
    rgb_to_bgr((139, 139, 0)), rgb_to_bgr((255, 20, 147)), rgb_to_bgr((75, 0, 130)),  # First row --> Dark
    rgb_to_bgr((255, 0, 0)), rgb_to_bgr((0, 255, 0)), rgb_to_bgr((0, 0, 255)),  # Second row --> normal
    rgb_to_bgr((255, 255, 255)), rgb_to_bgr((255, 255, 255)), rgb_to_bgr((255, 255, 255))  # Third row --> faint
]

# Fill the 3x3 grid with different colors
for i in range(3):
    for j in range(3):
        top_left_x = start_x + i * (box_size + gap)
        top_left_y = start_y + j * (box_size + gap)
        bottom_right_x = top_left_x + box_size
        bottom_right_y = top_left_y + box_size
        color = colors[i * 3 + j]
        image[top_left_x:bottom_right_x, top_left_y:bottom_right_y] = color

# Save or display the image
cv2.imwrite('image_2.jpg', image)
# To display the image uncomment the following line:
cv2.imshow('Image_1', image)
cv2.waitKey(0)
cv2.destroyAllWindows()