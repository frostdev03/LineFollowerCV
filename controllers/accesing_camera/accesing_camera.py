from controller import Robot, Camera

# Initialize the Robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Initialize the Camera
camera = robot.getDevice('camera')
camera.enable(timestep)

# Main loop
while robot.step(timestep) != -1:
    # Capture image
    image = camera.getImage()

    # Print out the dimensions of the image
    width = camera.getWidth()
    height = camera.getHeight()
    print(f"Image dimensions: {width}x{height}")
    
    # Optionally, you can access pixel values from the image
    # For example, print the RGB values of the pixel in the center of the image
    center_x = width // 2
    center_y = height // 2
    r = camera.imageGetRed(image, width, center_x, center_y)
    g = camera.imageGetGreen(image, width, center_x, center_y)
    b = camera.imageGetBlue(image, width, center_x, center_y)
    print(f"Center pixel RGB values: R={r}, G={g}, B={b}")

    # To avoid overwhelming the output, you might want to add a small delay or limit the number of prints
    robot.step(1000)  # Wait for 1000 ms before capturing the next image
