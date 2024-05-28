"""class controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import numpy as np
import cv2 

def get_center(im):
    gray = cv2.cvtColor(im, cv2.COLOR_RGB2GRAY)
    w, h = gray.shape
    ret, thresh = cv2.threshold(gray, 127,255, cv2.THRESH_BINARY_INV)
    
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(im, contours, -1, (0,0,255), 5 )
    
    cX = 0
    cY = 0
    if len(contours)>0:
      c = contours[0]
      M = cv2.moments(c)
      if (M["m00"]!=0):
          cX = int(M["m10"] / M["m00"])
          cY = int(M["m01"] / M["m00"])
          cv2.circle(im, (cX,cY), 3, (0,255,0),-1)
          #cv2.circle(im, (cX+150,cY), 40, (0,255,255),-1)
          #cv2.circle(im, (cX-150,cY), 40, (0,255,255),-1)
          #cv2.line(im, (cX-250,0), (cX-250,h), (255,0,255), 25)
          #cv2.line(im, (cX+250,0), (cX+250,h), (255,0,255), 25)
    return  cX, cY, im
      
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

cam = robot.getDevice('camera')
cam.enable(timestep)

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)


# Main loop:
# - perform simulation steps until Webots is stopping the controller
v = 5
while robot.step(timestep) != -1:
       
    img = cam.getImageArray()
    img = np.asarray(img, dtype=np.uint8)
    img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    crop_img = cv2.flip(img, 1)
    
    cX,cY, im = get_center(crop_img)
    cv2.imshow("Image", im)
    cv2.waitKey(33)
    
    vl = v
    vr = v
    if (cX>90):
        #ruotare a sinistra
        vr = -v
    if (cX<40):
        #ruotare a sinistra
        vl = -v
        
    #print(cX,cY)
    left_motor.setVelocity(vl)
    right_motor.setVelocity(vr)
    


# Enter here exit cleanup code.