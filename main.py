#! /usr/bin/env python
import sys
import rospy
import math
import cv2

from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sensor_msgs.msg import Image

class MovementLoggingNode():

    def __init__(self):
        rospy.init_node("MovementLoggingNode", anonymous=False)

        # Tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")
        self.bridge = CvBridge()
 
        # What function to call when you ctrl + c   
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel=rospy.Publisher(
               '/cmd_vel_mux/input/teleop',
               Twist,
               queue_size=10)

        #subscribe depth
        self.image_sub = rospy.Subscriber("camera/depth/image_raw",Image,self.callback)
	#subscribe rgb  
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.callbackrgb)  
        
        # TurtleBot will stop if we don't keep telling it to move. 
        # How often should we tell it to move? 10 HZ?
        r = rospy.Rate(10)
        # Twist is a datatype for velocity
        self.move_cmd = Twist()
 
        # Let's go forward at 0.2 m/s
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = 0

        self.P = 0.0
        self.Q = 0.0
        self.Angler = 0.0
        self.objectGroup= np.zeros((128,96,3))
        
       
        
        #self.PreValue.PrePosX = self.pose.pose.position.x
        #self.PreValue.PrePosY = self.pose.pose.position.y
        #self.PreValue.PrePosZ = self.pose.pose.position.z
        #self.PreValue.PreOriX = self.pose.pose.orientation.x
        #self.PreValue.PreOriY = self.pose.pose.orientation.y
        #self.PreValue.PreOriZ = self.pose.pose.orientation.z
        #self.PreValue.PreOriW = self.pose.pose.orientation.w

        # rospy.spin() tells the program to not exit until you press 
        # ctrl + c.  If this wasn't there, it'd subscribe to 
        # /cmd_vel_mux/input/teleop/ then immediately exit (therefore 
        # stop "listening" to the thread).
        while not rospy.is_shutdown():
            
 
           # publish the velocity
            self.cmd_vel.publish(self.move_cmd)
           # wait for 0.01 seconds (10 HZ) and publish again
            r.sleep()

    def callbackrgb(self, data):
	    self.picture= self.bridge.imgmsg_to_cv2(data, "bgr8")

    def getColGroups(self, start, end, labels):
        rows, cols = labels.shape
        for col in range(start, end):
            for row in range(rows):
                print(labels[col][row])
        return labels

    def getGroups(self, bitwiseOr, startRow, objectGroup, cv_image ,depth_image):
        height, width = bitwiseOr.shape
        cv_image = cv2.resize(cv_image, (width,height))
        thresh_type = cv2.THRESH_BINARY + cv2.THRESH_OTSU
        _, bitwiseOr = cv2.threshold(bitwiseOr, 0, 255, thresh_type)
        n_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(bitwiseOr,
                                                                      connectivity=4)
        groupLen=[0]*n_labels                                                               
        averageCol=[[0]*3]*n_labels
        averageDepth=[0]*n_labels
        pixelGroupList= [[] for i in range(n_labels)]
        for i in range(height):
            for j in range(width):
                num = labels[i][j]
                averageCol[num]+=cv_image[i][j]
                averageDepth[num]+=depth_image[i][j]
                pixelGroupList[num].append((i,j))
             
                groupLen[num]+=1
        for i in range(n_labels):
            averageDepth[i] /= groupLen[i]
            averageCol[i] /= groupLen[i]
           
        #print(len(pixelGroupList))
      # print(len(pixelGroupList[6]))
        for i in range(height):
            for j in range(width):
                cv_image[i][j] = averageCol[labels[i][j]]
        
        shrink=(1)
        newwidth = int(width*shrink)
        newheight = int(height*shrink)
        cv_image =cv2.resize(cv_image, (newwidth, newheight))
        self.getColGroups(0, 60, labels)
        return cv_image

    def callback(self,data):
        self.objectGroup= np.zeros((128,96,3))
        cv_image = self.bridge.imgmsg_to_cv2(data,'32FC1')
        inputscale=1
        depth_image = cv2.resize(cv_image,(int(256*inputscale),int(inputscale*192)))
        cv_image_edge = np.uint8(depth_image*255)
        #extract edges
        
        #cv_image_edge = cv2.resize(cv_image_edge,(320,240))
        depth_image_edge = cv2.Canny(image=cv_image_edge, threshold1=50, threshold2=200)
        #cv2.imshow("depth", depth_image_edge)
        #cv2.waitKey(1)
        if(self.picture.any):
            rgb_image =cv2.resize(self.picture,(int(inputscale*256),int(inputscale*192)))
            rgb_edge = cv2.Canny(image=rgb_image, threshold1=50, threshold2=100)
            bitwiseOr = cv2.bitwise_or(rgb_edge, depth_image_edge)
            M = np.float32([
	        [1, 0, 0],
	        [0, 1, 1]])
            N = np.float32([
	        [1, 0, -1],
	        [0, 1, 0]])
            if(1):
                bitwiseOrShift = cv2.warpAffine(bitwiseOr, M, (bitwiseOr.shape[1],bitwiseOr.shape[0]))
                bitwiseOrShift1 = cv2.warpAffine(bitwiseOr, N, (bitwiseOr.shape[1],bitwiseOr.shape[0]))
                bitwiseOr = cv2.bitwise_or(bitwiseOr, bitwiseOrShift, bitwiseOrShift1)
            scale=inputscale*1
            bitwiseOr= 	cv2.resize(bitwiseOr,(int(256*scale),int(192*scale)))
            bitwiseOr= cv2.bitwise_not(bitwiseOr)
            objectGroup = self.getGroups(bitwiseOr, 0, self.objectGroup, rgb_image ,depth_image)
           
            objectGroup= 	cv2.resize(objectGroup,(1280,960))
            bitwiseOr= 	cv2.resize(bitwiseOr,(1280,960))
            cv2.imshow("group", objectGroup)
            #cv2.imshow("edges", bitwiseOr)
            cv2.waitKey(1)
        
        
      

    def CalculationFunction(self):
        dumb=1


    
    



    def shutdown(self):
       # Stop turtlebot
       rospy.loginfo("Stop TurtleBot")
 
       # A default Twist has linear.x of 0 and angular.z of 0. 
       # So it'll stop TurtleBot
       self.cmd_vel.publish(Twist())
       # Sleep just makes sure TurtleBot receives the stop command
       # prior to shutting
       # down the script
       rospy.sleep(1)


        
        

if __name__ == '__main__':
    MovementLoggingNode()
