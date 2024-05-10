####
#   Main purpse is Process A single frame to extract
#   line and frame mid points using image processing techniques
#  Written by Ibrahim Mammadli
#  Ros2 Humble
#  02/09/23
#
###


import cv2
from numpy import *
import numpy as np
import rclpy
from rclpy.node import Node 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class line_detection_and_following(Node):
    def __init__(self):
        super().__init__('Line_follower')
        self.subscriber = self.create_subscription(Image,'/my_robot_camera/image_raw',self.process_data,10)
        self.subscriber2=self.create_subscription(LaserScan,'/scan',self.laser_scan_values,40) #Laser subscriber
        self.bridge = CvBridge() # converting ros images to opencv data
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 40)#periodic publisher call
        timer_period = 0.2;self.timer = self.create_timer(timer_period, self.send_cmd_vel)
        self.velocity=Twist()
        self.error=0
        self.action=""
        self.regions={'right':[],'mid':[],'left':[]}


    ## Subscriber Callback function 
    def laser_scan_values(self,scan_data):
        ## We have 360 data points and we are dividing them in 3 regions
        ## we say if there is something in the region get the smallest value
        self.regions = {
        'right':   int(min(min(scan_data.ranges[0:120])  , 100)),
        'mid':     int(min(min(scan_data.ranges[120:240]), 100)),
        'left':    int(min(min(scan_data.ranges[240:360]), 100)),
         }
        print(self.regions['left']," / ",self.regions['mid']," / ",self.regions['right'])


    ## Publisher Call back     
    def send_cmd_vel(self):
        ## Supllying constant linear Velocity
        
        
        if(self.regions['left'] > 4  and self.regions['mid'] > 4   and self.regions['right'] > 4 ):
                    self.velocity.linear.x=0.2
        elif(self.regions['left'] < 4 and self.regions['mid'] < 4  and self.regions['right'] < 4 ):
                    self.velocity.linear.x=0.0
                    self.velocity.angular.z=0.0       

        if(self.error > 0 and self.regions['left'] > 4 and self.regions['mid'] > 4  and self.regions['right'] > 4  ):## go left
                self.velocity.angular.z=0.4
                self.action="Go Left"
        elif(self.error < 0 and self.regions['left'] > 4 and self.regions['mid'] > 4  and self.regions['right'] > 4  ):## go right
                 self.velocity.angular.z=-0.4
                 self.action="Go Right"
        else:
             print("Stopped")

        ## Publishing completed velocities
        self.publisher.publish(self.velocity)


    ## Subscriber Call Back
    def process_data(self, data): 
        frame = self.bridge.imgmsg_to_cv2(data) # performing conversion
           ##### Segmentation
        ## Defining the color Upper bound and Lower bound limits to extract 
        light_line = np.array([ 100,100,100])
        dark_line = np.array([200,200,200])
        mask = cv2.inRange(frame, light_line,dark_line)
            ##### Boundaries Extraction
        ## applying the canny edge dector function
        canny= cv2.Canny(mask,40,10)
        ## Cropping the image
        r1=150;c1=0
        img = canny[r1:r1+240,c1:c1+640]

        ##### Finding Line  Mid point
        ## Visually finding the perfect Row to fix for mid points
        ## by setting all values to be light
        edge=[]
        for i in range (639):
            if(img[160,i]==255):
                edge.append(i)
        print(edge)

        ## We only need two points # one from left line # second from right line 
        # When values are 4
        if(len(edge)==4):
            edge[0]=edge[0]
            edge[1]=edge[2]
            
        ## When Values are 3 and if they are 2 we donot need to process them
        ## If the values of pixels is greater then 5 then they are adjecent to eachother
        if(len(edge)==3):
            for i in range(len(edge)):
                if(edge[1]-edge[0] > 5): ## meaning idx(0) and idx(1) are ok [193, 506, 507 ]
                    edge[0]=edge[0]
                    edge[1]=edge[1]
                else:#[193, 194, 507 ]
                    edge[0]=edge[0]
                    edge[1]=edge[2]
        ## Creating a dummy values for un expected case
        if(len(edge) < 2):
            edge=[240,440]
        
        ## Apllying a white pixel to final line mid point found
        mid_area=(edge[1]-edge[0])
        mid_point= +edge[0] + (mid_area/2)
        img[160,int(mid_point)]=255

       
            ##### Finding Frame Mid point
        frame_mid = 639/2
         
        # Screen to see the output
        self.error=frame_mid-mid_point 

        ## More apparent mid frame pixel by setting upper and lower pixels to be white 
        img[160,int(frame_mid)]=255
        img[159,int(frame_mid)]=255
        img[161,int(frame_mid)]=255


        ## Writing on the Frame as output for better understanding
        f_image = cv2.putText(img, self.action, (100,100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,), 2, cv2.LINE_AA)


        frame = frame[307:477,185:595]
        cv2.imshow('Frame',frame)
        cv2.imshow('output image',f_image)
        cv2.waitKey(1)
        

    cv2.waitKey(0)
    cv2.destroyAllWindows()


def main(args=None):
  rclpy.init(args=args)
  image_subscriber = line_detection_and_following()
  rclpy.spin(image_subscriber)
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()