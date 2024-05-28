from controller import Robot, Camera, Motor
import cv2
import numpy as np

# Initialize the Robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Initialize the Camera
camera = robot.getDevice('camera')
camera.enable(timestep)

# Initialize Motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Get camera dimensions
width = camera.getWidth()
height = camera.getHeight()

# Desired dimensions for the resized image
resized_width = 1080
resized_height = 1080

# Create a named window for the display
cv2.namedWindow('e-puck Camera', cv2.WINDOW_NORMAL)

# Main loop
while robot.step(timestep) != -1:
    # Capture image
    image = camera.getImage()

    # Convert the Webots image to a format compatible with OpenCV
    # Webots image is a single string, need to convert it to a numpy array
    img = np.zeros((height, width, 3), np.uint8)
    for y in range(height):
        for x in range(width):
            r = camera.imageGetRed(image, width, x, y)
            g = camera.imageGetGreen(image, width, x, y)
            b = camera.imageGetBlue(image, width, x, y)
            img[y, x] = [b, g, r]

    # Resize the image
    resized_img = cv2.resize(img, (resized_width, resized_height))

    # Display the resized image using OpenCV
    cv2.imshow('e-puck Camera', resized_img)

    # Set motor velocities (example velocities, adjust as needed)
    left_motor.setVelocity(2.0)  # Set left motor speed
    right_motor.setVelocity(2.0)  # Set right motor speed

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the OpenCV window
cv2.destroyAllWindows()
