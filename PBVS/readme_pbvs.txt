Cylinder_tracking_PBVS

This is a ROS package that tracks a cylindrical block using a robot with a camera. It uses OpenCV to process the camera images and extract the cylinder from the background. The robot then moves towards the cylinder by computing the centroid of the cylinder and publishing a Twist message to move the robot.

Requirements
    ROS (Robot Operating System)
    OpenCV
    Numpy
Installation
    Clone this repository into your catkin workspace:
        cd ~/catkin_ws/src
        git clone https://github.com/ankitmehra31/camera_cylinder_pbvs.git
Build the package:
    cd ~/catkin_ws
    catkin_make
Source the setup file:
    source devel/setup.bash

Usage
    Launch the ball tracker node:
        roslaunch mobile_robot camera_cylinder_pbvs.launch

    The node subscribes to the /camera/rgb/image_raw topic to receive images from the robot's camera. It publishes Twist messages to the /cmd_vel topic to move the robot.

Parameters
    lower_black: the lower threshold for the black color in HSV format. Default is [0, 0, 0].
    upper_black: the upper threshold for the black color in HSV format. Default is [180, 255, 30].
    kernel: the kernel used for morphological operations. Default is np.ones((5, 5), np.uint8).

Credits
    This package was developed by Ankit Mehra.

License
    This project is licensed under the course of Vision and Learning based conrol(EEXX2) at IIT-Mandi License.