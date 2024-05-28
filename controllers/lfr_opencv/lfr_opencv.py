"""class controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import numpy as np
import cv2

def get_center(im):
    gray = cv2.cvtColor(im, cv2.COLOR_RGB2GRAY)
    # w, h = gray.shape
    ret, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(im, contours, -1, (0, 0, 255), 5)

    cX = 0
    cY = 0
    if len(contours) > 0:
        c = contours[0]
        M = cv2.moments(c)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.circle(im, (cX, cY), 3, (0, 255, 0), -1)
            # cv2.circle(im, (cX+150,cY), 40, (0,255,255),-1)
            # cv2.circle(im, (cX-150,cY), 40, (0,255,255),-1)
            # cv2.line(im, (cX-250,0), (cX-250,h), (255,0,255), 25)
            # cv2.line(im, (cX+250,0), (cX+250,h), (255,0,255), 25)
    return cX, cY, im

robot = Robot()
timestep = int(robot.getBasicTimeStep())

cam = robot.getDevice("camera")
cam.enable(timestep)


left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Main loop:
max_speed = 6.28

while robot.step(timestep) != -1:
    img = cam.getImageArray()
    img = np.asarray(img, dtype=np.uint8)
    img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    img = cv2.resize(img, (0, 0), fx=2.0, fy=2.0)
    crop_img = cv2.flip(img, 1)

    cX, cY, im = get_center(crop_img)
    cv2.imshow("Image", im)
    cv2.waitKey(33)
    
    width = im.shape[1]
    center_point = width // 2
    error = cX - center_point
    
    # Display the image and error value
    print(f"Error: {error}")
    cv2.imshow("Image", im)
    cv2.waitKey(33)

    left_speed = max_speed
    right_speed = max_speed
    if cX > 90:
        right_speed = -max_speed
    if cX < 40:
        left_speed = -max_speed

    # print(cX,cY)
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)


# Enter here exit cleanup code.
