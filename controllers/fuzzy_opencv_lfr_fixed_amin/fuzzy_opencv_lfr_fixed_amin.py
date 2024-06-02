from controller import Robot
from controller import Display
import numpy as np
import cv2
import skfuzzy as fuzz
import skfuzzy.control as ctl

def get_center(im):
    gray = cv2.cvtColor(im, cv2.COLOR_RGB2GRAY)
    # ret, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)
    # contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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
    return cX, cY, im

# Define fuzzy system
# Input
error = ctl.Antecedent(np.arange(-160, 161, 1), 'Error')
delta_error = ctl.Antecedent(np.arange(-160, 161, 1), 'Delta Error')

# Output
left_motor_speed = ctl.Consequent(np.arange(0, 6.29, 0.1), 'Left Motor Speed')
right_motor_speed = ctl.Consequent(np.arange(0, 6.29, 0.1), 'Right Motor Speed')

# Membership functions for error
error['highly_negative'] = fuzz.trimf(error.universe, [-160, -160, -80])
error['negative'] = fuzz.trimf(error.universe, [-160, -80, 0])
error['mid'] = fuzz.trimf(error.universe, [-80, 0, 80])
error['positive'] = fuzz.trimf(error.universe, [0, 80, 160])
error['highly_positive'] = fuzz.trimf(error.universe, [80, 160, 160])

# Membership functions for delta_error
delta_error['highly_negative'] = fuzz.trimf(delta_error.universe, [-160, -160, -80])
delta_error['negative'] = fuzz.trimf(delta_error.universe, [-160, -80, 0])
delta_error['mid'] = fuzz.trimf(delta_error.universe, [-80, 0, 80])
delta_error['positive'] = fuzz.trimf(delta_error.universe, [0, 80, 160])
delta_error['highly_positive'] = fuzz.trimf(delta_error.universe, [80, 160, 160])

# Membership functions for motor speed
left_motor_speed['very_slow'] = fuzz.trimf(left_motor_speed.universe, [0, 0, 1.57])
left_motor_speed['slow'] = fuzz.trimf(left_motor_speed.universe, [0.79, 1.57, 3.14])
left_motor_speed['medium'] = fuzz.trimf(left_motor_speed.universe, [1.57, 3.14, 4.71])
left_motor_speed['fast'] = fuzz.trimf(left_motor_speed.universe, [3.14, 4.71, 6.28])
left_motor_speed['very_fast'] = fuzz.trimf(left_motor_speed.universe, [4.71, 6.28, 6.28])

right_motor_speed['very_slow'] = fuzz.trimf(right_motor_speed.universe, [0, 0, 1.57])
right_motor_speed['slow'] = fuzz.trimf(right_motor_speed.universe, [0.79, 1.57, 3.14])
right_motor_speed['medium'] = fuzz.trimf(right_motor_speed.universe, [1.57, 3.14, 4.71])
right_motor_speed['fast'] = fuzz.trimf(right_motor_speed.universe, [3.14, 4.71, 6.28])
right_motor_speed['very_fast'] = fuzz.trimf(right_motor_speed.universe, [4.71, 6.28, 6.28])

# Define rules
rules = [
    ctl.Rule(error['highly_negative'] & delta_error['highly_negative'], (left_motor_speed['very_fast'], right_motor_speed['very_slow'])),
    ctl.Rule(error['highly_negative'] & delta_error['negative'], (left_motor_speed['fast'], right_motor_speed['slow'])),
    ctl.Rule(error['highly_negative'] & delta_error['mid'], (left_motor_speed['medium'], right_motor_speed['slow'])),
    ctl.Rule(error['highly_negative'] & delta_error['positive'], (left_motor_speed['medium'], right_motor_speed['medium'])),
    ctl.Rule(error['highly_negative'] & delta_error['highly_positive'], (left_motor_speed['slow'], right_motor_speed['fast'])),

    ctl.Rule(error['negative'] & delta_error['highly_negative'], (left_motor_speed['fast'], right_motor_speed['very_slow'])),
    ctl.Rule(error['negative'] & delta_error['negative'], (left_motor_speed['medium'], right_motor_speed['slow'])),
    ctl.Rule(error['negative'] & delta_error['mid'], (left_motor_speed['medium'], right_motor_speed['medium'])),
    ctl.Rule(error['negative'] & delta_error['positive'], (left_motor_speed['slow'], right_motor_speed['medium'])),
    ctl.Rule(error['negative'] & delta_error['highly_positive'], (left_motor_speed['slow'], right_motor_speed['fast'])),

    ctl.Rule(error['mid'] & delta_error['highly_negative'], (left_motor_speed['medium'], right_motor_speed['fast'])),
    ctl.Rule(error['mid'] & delta_error['negative'], (left_motor_speed['medium'], right_motor_speed['medium'])),
    ctl.Rule(error['mid'] & delta_error['mid'], (left_motor_speed['medium'], right_motor_speed['medium'])),
    ctl.Rule(error['mid'] & delta_error['positive'], (left_motor_speed['medium'], right_motor_speed['medium'])),
    ctl.Rule(error['mid'] & delta_error['highly_positive'], (left_motor_speed['medium'], right_motor_speed['fast'])),

    ctl.Rule(error['positive'] & delta_error['highly_negative'], (left_motor_speed['slow'], right_motor_speed['medium'])),
    ctl.Rule(error['positive'] & delta_error['negative'], (left_motor_speed['slow'], right_motor_speed['medium'])),
    ctl.Rule(error['positive'] & delta_error['mid'], (left_motor_speed['medium'], right_motor_speed['medium'])),
    ctl.Rule(error['positive'] & delta_error['positive'], (left_motor_speed['slow'], right_motor_speed['fast'])),
    ctl.Rule(error['positive'] & delta_error['highly_positive'], (left_motor_speed['very_slow'], right_motor_speed['very_fast'])),

    ctl.Rule(error['highly_positive'] & delta_error['highly_negative'], (left_motor_speed['slow'], right_motor_speed['fast'])),
    ctl.Rule(error['highly_positive'] & delta_error['negative'], (left_motor_speed['slow'], right_motor_speed['medium'])),
    ctl.Rule(error['highly_positive'] & delta_error['mid'], (left_motor_speed['medium'], right_motor_speed['medium'])),
    ctl.Rule(error['highly_positive'] & delta_error['positive'], (left_motor_speed['slow'], right_motor_speed['fast'])),
    ctl.Rule(error['highly_positive'] & delta_error['highly_positive'], (left_motor_speed['very_slow'], right_motor_speed['very_fast']))
]

# Control system
left_motor_speed_ctrl = ctl.ControlSystem(rules)
right_motor_speed_ctrl = ctl.ControlSystem(rules)

left_motor_speed_sim = ctl.ControlSystemSimulation(left_motor_speed_ctrl)
right_motor_speed_sim = ctl.ControlSystemSimulation(right_motor_speed_ctrl)

# Initialize Robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

cam = robot.getDevice("camera")
# cam.setWidth(128)
# cam.setHeight(128)
cam.enable(timestep)

# display = Display("display")
# display.attachCamera(cam)

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Main loop:
previous_error_value = 0
max_speed = 6.28

while robot.step(timestep) != -1:
    img = cam.getImageArray()
    img = np.asarray(img, dtype=np.uint8)
    img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    img = cv2.resize(img, (0, 0), fx=3.5, fy=3.5)
    crop_img = cv2.flip(img, 1)

    cX, cY, im = get_center(crop_img)
    # display.imagePaste(im, 0, 0, False)
    cv2.imshow("Image", im)
    cv2.waitKey(33)
    
    width = im.shape[1]
    center_point = width // 2
    error_value = cX - center_point
    delta_error_value = error_value - previous_error_value
    previous_error_value = error_value

    # Fuzzy inference
    left_motor_speed_sim.input['Error'] = error_value
    left_motor_speed_sim.input['Delta Error'] = delta_error_value
    right_motor_speed_sim.input['Error'] = error_value
    right_motor_speed_sim.input['Delta Error'] = delta_error_value

    left_motor_speed_sim.compute()
    right_motor_speed_sim.compute()
    
    print(f"Error: {error_value}")
    print(f"Delta Error: {delta_error_value}")

    left_motor_speed_output = left_motor_speed_sim.output['Left Motor Speed']
    right_motor_speed_output = right_motor_speed_sim.output['Right Motor Speed']

    # Set motor velocities
    left_motor.setVelocity(left_motor_speed_output)
    right_motor.setVelocity(right_motor_speed_output)

# Exit cleanup
cv2.destroyAllWindows()
