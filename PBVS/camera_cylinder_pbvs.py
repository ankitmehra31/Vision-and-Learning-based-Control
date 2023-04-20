#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class CylinderTracking:
    def __init__(self):
        
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Initialize image processing parameters

        self.lower_black = np.array([0, 0, 0])
        self.upper_black = np.array([180, 255, 30])
        self.kernel = np.ones((5, 5), np.uint8)

        # Initialize PBVS parameters
        self.image_width = 640
        self.image_height = 480
        self.focal_length = 500
        self.object_width = 0.5
        self.object_height=1
        self.desired_distance = 0.5

    def image_callback(self, msg):

        # Convert ROS image message to OpenCV image
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Apply color thresholding to extract black cylinder
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_black, self.upper_black)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(gray,150,255,0)
        thresh = cv2.bitwise_not(thresh)

        # Find contours in the mask
        contours,hierarchy = cv2.findContours(thresh, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        print(len(contours))
        cnt = contours[0]

        # compute the area and perimeter
        area = cv2.contourArea(cnt)
        print('Area:', area)

        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        width=w
        height=h
        # Compute the centroid of the contour
        M = cv2.moments(largest_contour)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        # Showing the contour
        img1 = cv2.drawContours(img, [cnt], -1, (0,255,255), 3)
        x1, y1 = cnt[0,0]
        cv2.putText(img1, f'Area:{area}', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.imshow("output",img1)
        cv2.waitKey(1)
        # Compute the error in the x-axis
        error_x = cx - self.image_width / 2

        depth=self.object_height*self.focal_length/height # Formula calculated using similar traingles

        # Compute the error in the depth
        error_depth = depth - self.desired_distance

        error = cx - img.shape[1] / 2

        # Publish a twist message to move the robot

        print("error is ",error,error_depth)
        twist = Twist()
        twist.linear.x = 0.02 * error_depth
        twist.angular.z = -0.02 * error_x / self.focal_length
        self.cmd_vel_pub.publish(twist)

    def run(self):
    # Spin until shutdown
        rospy.spin()
      
if __name__ == '__main__':
    rospy.init_node('ball_tracker')
    tracker = CylinderTracking()
    tracker.run()
