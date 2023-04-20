Obstacle_avoidance_IBVS

This is a ROS package that senses a cylindrical obstacle using a robot with a camera. It uses OpenCV to process the camera images and extract the cylindrical pole from the background. The robot then moves towards the cylinder by computing the area of the cylinder and publishing a Twist message to move the robot.

Requirements
    ROS (Robot Operating System)
    OpenCV
    Numpy
Installation
    Clone this repository into your catkin workspace:
        cd ~/catkin_ws/src
        git clone https://github.com/ankitmehra31/camera_sensor_ibvs.git
Build the package:
    cd ~/catkin_ws
    catkin_make
Source the setup file:
    source devel/setup.bash

Usage
    Launch the ball tracker node:
        roslaunch mobile_robot camera_sensor_ibvs.launch

    The node subscribes to the /camera/rgb/image_raw topic to receive images from the robot's camera. It publishes Twist messages to the /cmd_vel topic to move the robot.

Parameters
    thresh: Applying thresholding in the gray image to create a binary image.
    cv2.drawContours: Draws a contour arond the largest contour in the image plane.

Credits
    This package was developed by Ankit Mehra.

License
    This project is licensed under the course of Vision and Learning based conrol(EEXX2) at IIT-Mandi License.