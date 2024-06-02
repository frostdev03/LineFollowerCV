from controller import Robot
import numpy as np
import cv2
import skfuzzy as fuzz
import skfuzzy.control as ctl

def get_center(im):
    gray = cv2.cvtColor(im, cv2.COLOR_RGB2GRAY)
    # w, h = gray.shape
    ret, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # cv2.drawContours(im, contours, -1, (0, 0, 255), 5)

    cX = 0
    cY = 0
    height, width = im.shape[:2]
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
            
                # Center line
        center_x = width // 2
        cv2.line(im, (center_x, 0), (center_x, height), (255, 0, 0), 2)
    return cX, cY, im

# Define fuzzy system
# Input
error = ctl.Antecedent(np.arange(-160, 161, 1), 'Error')
delta_error = ctl.Antecedent(np.arange(-160, 161, 1), 'Delta Error')

# Output
rms = ctl.Consequent(np.arange(0, 6.29, 0.1), 'RMS')
lms = ctl.Consequent(np.arange(0, 6.29, 0.1), 'LMS')

# Membership functions for error
error['negative'] = fuzz.trimf(error.universe, [-160, -80, 0])
error['zero'] = fuzz.trimf(error.universe, [-80, 0, 80])
error['positive'] = fuzz.trimf(error.universe, [0, 80, 160])

# Membership functions for delta_error
delta_error['negative'] = fuzz.trimf(delta_error.universe, [-160, -80, 0])
delta_error['zero'] = fuzz.trimf(delta_error.universe, [-80, 0, 80])
delta_error['positive'] = fuzz.trimf(delta_error.universe, [0, 80, 160])

# Membership functions for motor speed
rms['slow'] = fuzz.trimf(rms.universe, [0, 0, 3.14])
rms['medium'] = fuzz.trimf(rms.universe, [1.57, 3.14, 4.71])
rms['fast'] = fuzz.trimf(rms.universe, [3.14, 6.28, 6.28])

lms['slow'] = fuzz.trimf(lms.universe, [0, 0, 3.14])
lms['medium'] = fuzz.trimf(lms.universe, [1.57, 3.14, 4.71])
lms['fast'] = fuzz.trimf(lms.universe, [3.14, 6.28, 6.28])

# Define rules
rule1 = ctl.Rule(error['negative'] & delta_error['negative'], (rms['fast'], lms['slow']))
rule2 = ctl.Rule(error['negative'] & delta_error['zero'], (rms['medium'], lms['slow']))
rule3 = ctl.Rule(error['negative'] & delta_error['positive'], (rms['slow'], lms['slow']))
rule4 = ctl.Rule(error['zero'] & delta_error['negative'], (rms['fast'], lms['medium']))
rule5 = ctl.Rule(error['zero'] & delta_error['zero'], (rms['medium'], lms['medium']))
rule6 = ctl.Rule(error['zero'] & delta_error['positive'], (rms['slow'], lms['medium']))
rule7 = ctl.Rule(error['positive'] & delta_error['negative'], (rms['fast'], lms['fast']))
rule8 = ctl.Rule(error['positive'] & delta_error['zero'], (rms['medium'], lms['fast']))
rule9 = ctl.Rule(error['positive'] & delta_error['positive'], (rms['slow'], lms['fast']))

# Control system
rms_ctrl = ctl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9])
lms_ctrl = ctl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9])

# Simulation
rms_sim = ctl.ControlSystemSimulation(rms_ctrl)
lms_sim = ctl.ControlSystemSimulation(lms_ctrl)

# Initialize Robot
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
previous_error = 0
max_speed = 6.28

while robot.step(timestep) != -1:
    img = cam.getImageArray()
    img = np.asarray(img, dtype=np.uint8)
    img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    img = cv2.resize(img, (0, 0), fx=3.5, fy=3.5)
    crop_img = cv2.flip(img, 1)

    cX, cY, im = get_center(crop_img)
    cv2.imshow("Image", im)
    cv2.waitKey(33)
    
    width = im.shape[1]
    center_point = width // 2
    error = cX - center_point
    delta_error = error - previous_error
    previous_error = error

    # Fuzzy inference
    rms_sim.input['Error'] = error
    rms_sim.input['Delta Error'] = delta_error
    lms_sim.input['Error'] = error
    lms_sim.input['Delta Error'] = delta_error

    rms_sim.compute()
    lms_sim.compute()
    
    print(f"Error: {error}")
    print(f"Delta Error: {delta_error}")

    right_speed = rms_sim.output['RMS']
    left_speed = lms_sim.output['LMS']

    # Set motor velocities
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    
    # Debug
    print("=------------------------------=")
    print(f"Center point: {center_point}")
    print(f"Contour center X: {cX}")
    print(f"Error: {error}")
    print(f"Delta Error: {delta_error}")
    print(f"Left Motor Speed: {left_speed}")
    print(f"Right Motor Speed: {right_speed}")

# Exit cleanup
cv2.destroyAllWindows()