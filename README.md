# Line Follower Robot

![WhatsApp Image 2024-06-02 at 8 06 38 PM](https://github.com/frostdev03/LineFollowerRobotWithCamera/assets/77367592/793c2230-d276-4fba-8cb5-ec62210725e6)

## Project Description
This project implements fuzzy control to adjust the speed of the right and left motors based on position error and change in position error in a line follower robot. This robot is able to follow a predetermined line path by relying on the camera to detect the center of the line (arena) and adjust the motor speed to keep the robot stable and keep on track.

## Main Features
- Fuzzy Control: Uses fuzzy control to adjust motor speed based on error and change in error (delta error).
- Line Center Detection: Uses OpenCV to process images from the camera and detect the line center.
- Data Storage: Saved the error, delta error, and motor speed data (LMS and RMS) into a CSV file for further analysis.
- Data Visualization and Analysis: Plot graphs of error, delta error, LMS, and RMS and calculate and display rise time, settling time, average error, delta error, and error mode.
- Robot Movement Map: Display the robot movement map for movement pattern analysis.

## Before running this program you must install
- Python 3.6 or higher
- NumPy
- OpenCv
- Matplotlib
- Scikit-Fuzzy
- Webots

## How to use
- Clone repository
  ```
  $ git clone https://github.com/frostdev03/LineFollowerRobotWithCamera
  $ cd LineFollowerRobotWithCamera
  ```
- Install dependency
  ```
  $ pip install -r requirements
  ```
- Select the required controller
- Run 

## Project Structure
- lfr_cam_fuzzy.py: The main script to control the line follower robot.
- robot_readings.csv: CSV file to store error, delta error, and motor speed data.
- olah_data.ipynb: Script to visualize and analyze the collected data.
- README.md: Project description.

## Useful Links
- https://www.neliti.com/id/publications/66722/penerapan-logika-fuzzy-dan-pulse-width-modulation-untuk-sistem-kendali-kecepatan
- https://www.youtube.com/watch?v=D0jhvFZJ5Ok
- https://youtu.be/ZiRJDt-Wo8E?si=TYKJqeapcs8LqrDk
- https://youtu.be/fRqwrIbjFTs
- https://youtu.be/UieEEyVY2J4?si=aYjkGU8v1GYzxi4Q
- https://youtu.be/HV6_usN4unc?si=AM7EdT1PBQ4XA-hd
- https://www.youtube.com/watch?v=nLnp0tpZ0ok
- https://webots.cloud/proto?keyword=primitive
