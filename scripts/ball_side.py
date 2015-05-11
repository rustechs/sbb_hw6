#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

class ballSide:

	def __init__(self,side):
		# Set HSV color constants
		GREEN = np.array([70,40,80])
		BALL = np.array([165,30,10])
		RANGE = np.array([10,20,20])

		# Subscribe to head camera frame topic
	    self.frame_sub = rospy.Subscriber('camera/rgb/image_color',Image,self.parseFrame)
	    # Publish parsed frames to debug topic
	    self.frame_pub = rospy.Publisher('sbb/cv/head_cam_overlay',Image,queue_size=10)
	    # Publish parsing to topic for controller
	    self.ball_pub = rospy.Publisher('sbb/ball_side',Bool,queue_size=10)

	    # Create bridge to convert between CV and ROS images
	    self.br = CvBridge()

	    # Set what side we're on
	    self.side = side

	    # Keep track of last side ball was detected on
	    self.ballSide = None

	# Camera frame topic callback
	# Parse the frame, look for ball, and publish result to another topic 
	def parseFrame(self,data):
	    # Convert image into openCV format
	    try:
	        self.img = br.imgmsg_to_cv2(data, "bgr8")
	    except CvBridgeError, e:
	        print e
	        raise

	    # Get frame properties -- width, height, and center
	    self.w = int(self.img.shape[1])
	    self.h = int(self.img.shape[0])

	    # Convert image to HSV
	    self.imgHSV = cv2.cvtColor(self.img,cv2.COLOR_BGR2HSV)

	    # Perform saturation thresholding
	    # ???

	    # Find x coordinate of field center
	    self.centerX = getCenter()

	    # Mask off opponent side
	   	self.ballLoc = findBall()

	   	# If ball was found, check what side it's on
	   	# Otherwise, publish previous value
	   	if self.ballLoc is not None:
	   		
	    self.ball_pub.publish(ourSide)

	    # publish debug overlay image
	    self.pubDebugImg()

	def pubDebugImg(self):
		self.frame_pub.publish(self.br.cv2_to_imgmsg(self.img, "bgr8"))

	def getCenter(self):

		MIN = GREEN-RANGE/2
		MAX = GREEN+RANGE/2

		# Threshold for green center field
        img = cv2.inRange(self.imgHSV,MIN,MAX)

		# Set up ROI
        ROIhorz = 0.25
        ROIvert = 0.9
        
        leftW = int(self.w*(1-ROIhorz)/2)
        rightW = int(self.w[1]*(0.5+ROIhorz/2))
        topW = int(self.h[0]*(1-ROIvert)/2)
        botW = int(self.h[0]*(0.5+ROIvert/2))

        # Mask the ROI
        img[:,:leftW] = 0
        img[:,rightW:] = 0
        img[:topW,:] = 0
        img[botW:,:] = 0

        # Find the right contour
        c,h = cv2.findContours(img,1,2)
        
        maxcArea = 0
        for i in range(len(c)):
             cAreas = cv2.contourArea(c[i])
             if maxcArea<cAreas:
                maxcArea = cAreas
                indexcArea = i
          
        cnt = c[indexcArea]               # Usually contours with maximum area corresponds to the block 

        rect = cv2.minAreaRect(cnt)       # Minimum fitted ractangle to the blue block

        x,y = rect[0]

        return int(x)

    def findBall(self):

        #!!! Need to measure color for the orange ball
        MIN = np.array([25,60,10])
        MAX = np.array([85,180,115])

        self.imgThreshBall = cv2.inRange(self.imgHSV,MIN,MAX)

        # Check if there's enough orange pixels to constitute a ball
        if float(cv2.countNonZero(self.imgThreshBall))/(self.img.shape[0]*self.img.shape[1]) >= 0.002:

            # Old Naive method
            # m = cv2.moments(self.imgThreshBall)
            # x = int(m['m10']/m['m00'])
            # y = int(m['m01']/m['m00'])
            # dx = x - self.center[0]
            # dy = self.center[1] - y

            # Find the right contour
            c,h = cv2.findContours(self.imgThreshBall,1,2)
            
            maxcArea = 0
            for i in range(len(c)):
                 cAreas = cv2.contourArea(c[i])
                 if maxcArea<cAreas:
                    maxcArea = cAreas
                    indexcArea = i
              
            cnt = c[indexcArea]  # Usually contours with maximum area corresponds to the ball 

            (x,y),radius = cv2.minEnclosingCircle(cnt)
            center = (int(x),int(y))
            radius = int(radius)
            # img = cv2.circle(img,center,radius,(0,255,0),2)  #DB PLot

            dx = center[0] - self.center[0]
            dy = self.center[1] - center[1]
            return (True,dx,dy,0)
        else:
            return (False,0,0,0)

def main():

	# Init node
    rospy.init_node('ball_side', anonymous = True)

    rospy.loginfo("Waiting on team side assignment")

    # Wait for side parameter to exist
    while not rospy.has_param('sbb_side'):
    	pass

    # Create ballSide object
    bs = ballSide(rospy.get_param('sbb_side'))

    ### Everything ready to go! ###

    rospy.loginfo("ball_side node started!")

    # Wait for a new frame on the camera topic
    rospy.spin()

if __name__ == "__main__":
	main()
