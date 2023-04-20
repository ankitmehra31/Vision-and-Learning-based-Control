#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from cv_bridge import CvBridge
import cv2

class sensor_checking:

    def __init__(self):
        sub_topic_name="/camera/rgb/image_raw"
        self.camera_subscriber=rospy.Subscriber(sub_topic_name,Image,self.camera_callback)
        self.speed_pub=rospy.Publisher("/cmd_vel",Twist,queue_size=1)
        self.bridge=CvBridge()

    def camera_callback(self,data):
        
        frame=self.bridge.imgmsg_to_cv2(data)
        edge_frame=cv2.Canny(frame,100,200 )

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


        # Apply thresholding in the gray image to create a binary image

        ret,thresh = cv2.threshold(gray,150,255,0)
        thresh = cv2.bitwise_not(thresh)
        
        # Find the contours using binary image

        contours,hierarchy = cv2.findContours(thresh, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        print("Number of contours in image:",len(contours))
        cnt = contours[0]

        # compute the area 

        area = cv2.contourArea(cnt)
        print('Area:', area)
        img1 = cv2.drawContours(frame, [cnt], -1, (0,255,255), 3)
        x1, y1 = cnt[0,0]
        cv2.putText(img1, f'Area:{area}', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.imshow("output",img1)
        cv2.waitKey(1) 
        
        #Calculating Error 
        error_z=48000-area   # Value 48000 is chosen by demonstration.
        print("Error is : ",error_z)
        
        #Publishing appropriate speed
        speed_cmd=Twist();
        speed_cmd.linear.x=error_z/100000;   #Using Propotional controller

        self.speed_pub.publish(speed_cmd)
    
if __name__=="__main__":
    node_name="sensor_check"
    rospy.init_node(node_name)
    sensor_checking()
    rospy.spin()