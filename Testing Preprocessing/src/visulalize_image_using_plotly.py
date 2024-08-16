import cv2
import plotly.express as px

image_path = '/Users/user/Documents/ROS_new_potato/Testing Preprocessing/resources/images/output_step_1.jpg'
image = cv2.imread(image_path) # will be read BGR by default.

image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

image_shape = image.shape

print(f"shape of image: {image_shape}")

fig = px.imshow(image_rgb)
fig.show()

