import cv2
import plotly.express as px

stiched_image = cv2.imread('/Users/user/Documents/ROS_new_potato/Testing Preprocessing/resources/stitched_image.jpg')

image = cv2.cvtColor(stiched_image, cv2.COLOR_BGR2RGB)


shape = stiched_image.shape
print(f"Shape before padding: {shape}")

top = 30
bottom = 30
left = 30
right = 30
padded_image = cv2.copyMakeBorder(image, top, bottom, left, right, cv2.BORDER_CONSTANT, value = [0, 255, 0])

image2 = cv2.cvtColor(padded_image, cv2.COLOR_BGR2RGB)

shape2 = image2.shape
print(f"Shape after padding: {shape2}")

output_path = '/Users/user/Documents/ROS_new_potato/Testing Preprocessing/resources/padded_image.jpg'
cv2.imwrite(output_path, image2)

fig = px.imshow(padded_image)
fig.show()


