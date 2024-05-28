from controller import Robot, Camera, Motor

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

# Function to read and process image from the camera
def get_line_position():
    image = camera.getImageArray()
    width = camera.getWidth()
    height = camera.getHeight()

    line_position = 0
    line_found = False

    for x in range(width):
        if image[height // 2][x][0] < 128:
            line_position += x
            line_found = True

    if not line_found:
        return None
    return line_position // width

# Fuzzy control function
def fuzzy_control(error, delta_error):
    adjust_speed = -0.1 * error
    return adjust_speed, -adjust_speed

# Main loop
prev_error = 0
while robot.step(timestep) != -1:
    line_position = get_line_position()

    if line_position is not None:
        error = line_position - 0.5
        delta_error = error - prev_error
        prev_error = error

        left_adjust, right_adjust = fuzzy_control(error, delta_error)
        left_speed = 1.0 + left_adjust
        right_speed = 1.0 + right_adjust

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
    else:
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
