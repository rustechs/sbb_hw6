#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

class ballSide:

    # Set HSV color constants
    GREEN_MIN = np.array([62,80,190])
    GREEN_MAX = np.array([78,125,230])
    BALL_MIN = np.array([160,190,180])
    BALL_MAX = np.array([180,255,255])
    BALL_RAD_MIN = 0.013
    BALL_RAD_MAX = 0.016
    BALL_MIN_BGR = np.array([0,0,50])
    BALL_MAX_BGR = np.array([100,50,255])
    centerW = 0.04

    def __init__(self,side):

        # Subscribe to head camera frame rgb topic
        self.frame_sub = rospy.Subscriber('camera/rgb/image_color',Image,self.parseFrame)
        # Subscribe to head camera frame depth topic
        # self.frame_depth_sub = rospy.Subscriber('camera/depth/image',Image,self.grabDepth)
        # Publish parsed frames to debug topic
        self.frame_pub = rospy.Publisher('sbb/cv/head_cam_overlay',Image,queue_size=10)
        # Publish parsing to topic for controller
        self.ball_pub = rospy.Publisher('sbb/ball_side',Bool,queue_size=10)

        # Create bridge to convert between CV and ROS images
        self.br = CvBridge()

        # Set what side we're on
        self.side = side

        # Keep track of last side ball was detected on
        self.ourBall = False

        # Latest depth image
        self.depth = None

        # Latest bgr image
        self.img = None

    # Camera depth frame topic callback
    def grabDepth(self,data):
        # Convert image into openCV format
        try:
            self.depth = self.br.imgmsg_to_cv2(data, "mono8")
            import pdb; pdb.set_trace()
        except CvBridgeError, e:
            print e
            raise

    # Camera rgb frame topic callback
    # Parse the frame, look for ball, and publish result to another topic 
    def parseFrame(self,data):
        # Convert image into openCV format
        try:
            self.img = self.br.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e
            raise

        # Get frame properties -- width, height, and center
        self.w = int(self.img.shape[1])
        self.h = int(self.img.shape[0])

        # Convert image to HSV
        self.imgHSV = cv2.cvtColor(self.img,cv2.COLOR_BGR2HSV)

        # Find x coordinate of field center
        self.centerX = self.getCenter()

        if self.centerX is None:
            rospy.logerr("Can't find field center!")

        # Find ball in field
       	self.ballLoc = self.findBall()

       	# If ball was found
       	if self.ballLoc is not None and self.centerX is not None:
            rospy.loginfo("Last Known Ball Loc: %s" % str(self.ballLoc))
            if self.side is 'left': # If we're on the left
                # Ball has moved to our side since last update
                if (self.ballLoc[0] >= self.centerX+ballSide.centerW) and (self.ourBall is False):
                    self.ourBall = True
                    self.ball_pub.publish(self.ourBall)
                    rospy.loginfo("Ball is on our side now")
                # Ball has moved to opponent side since last update
                elif (self.ballLoc[0] < self.centerX-ballSide.centerW) and (self.ourBall is True):
                    self.ourBall = False
                    self.ball_pub.publish(self.ourBall)
                    rospy.loginfo("Ball is on opponent now")
            else: # If we're on the right
                # Ball has moved to our side since last update
                if (self.ballLoc[0] < self.centerX-ballSide.centerW) and (self.ourBall is False):
                    self.ourBall = True
                    self.ball_pub.publish(self.ourBall)
                    rospy.loginfo("Ball is on our side now")
                # Ball has moved to opponent side since last update
                elif (self.ballLoc[0] > self.centerX+ballSide.centerW) and (self.ourBall is True):
                    self.ourBall = False
                    self.ball_pub.publish(self.ourBall)
                    rospy.loginfo("Ball is on opponent now")

        self.pubDebugImg()

    # Publish debug image to debug topic
    def pubDebugImg(self):
        imgShow = self.img

        # Show masked center field image
        # imgShowField = cv2.bitwise_and(imgShow,imgShow,mask = self.centerMask)

        # Show masked ball image
        imgShow = cv2.bitwise_and(imgShow,imgShow,mask = self.imgThreshBall)

        # imgShow = cv2.bitwise_or(imgShowBall,imgShowField)

        if self.centerX is not None:
            # Plot a center line if it's found
            cv2.line(imgShow,(self.centerX,0),(self.centerX,self.h),(30,30,240),1)

        # If ball was found, show it
        if self.ballLoc is not None:
            cv2.circle(imgShow,(self.ballLoc[0],self.ballLoc[1]),7,(30,240,30),-1)

        self.frame_pub.publish(self.br.cv2_to_imgmsg(imgShow, "bgr8"))

    def getCenter(self):
        # Threshold for green center field
        img = cv2.inRange(self.imgHSV,ballSide.GREEN_MIN,ballSide.GREEN_MAX)

        # Set up ROI
        ROIhorz = 0.25
        ROIvert = 0.9
        
        leftW = int(self.w*(1-ROIhorz)/2)
        rightW = int(self.w*(0.5+ROIhorz/2))
        topW = int(self.h*(1-ROIvert)/2)
        botW = int(self.h*(0.5+ROIvert/2))

        # Mask the ROI
        img[:,:leftW] = 0
        img[:,rightW:] = 0
        img[:topW,:] = 0
        img[botW:,:] = 0

        ballSide.centerMask = img

        m = cv2.moments(img)
        return int(m['m10']/m['m00'])

    def findBall(self):

        # Threshold image in HSV space for reddish colors
        imgHSVMask = cv2.inRange(self.imgHSV,ballSide.BALL_MIN,ballSide.BALL_MAX)

        # Use HSV threshold to mask BGR image
        imgBGR = cv2.bitwise_and(self.img,self.img,mask = imgHSVMask)

        # Threshold BGR image 
        img = cv2.inRange(imgBGR,ballSide.BALL_MIN_BGR,ballSide.BALL_MAX_BGR)

        # Use Kinect depth to threshold away remaining arm garbage

        # Set up ROI
        ROIhorz = 0.85
        ROIvert = 0.7
        
        leftW = int(self.w*(1-ROIhorz)/2)
        rightW = int(self.w*(0.5+ROIhorz/2))
        topW = int(self.h*(1-ROIvert)/2)
        botW = int(self.h*(0.5+ROIvert/2))

        # Mask the ROI
        img[:,:leftW] = 0
        img[:,rightW:] = 0
        img[:topW,:] = 0
        img[botW:,:] = 0

        self.imgThreshBall = img

        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()
         
        # Change thresholds
        params.minThreshold = 0;
        params.maxThreshold = 255;
         
        # Filter by Area.
        params.filterByArea = True
        params.minArea = 12
        params.maxArea = 100
         
        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.4
         
        # Filter by Convexity
        params.filterByConvexity = False
        params.minConvexity = 0.25
        params.maxConvexity = 1

        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0.2
         
        # Create a detector with the parameters
        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3 :
            detector = cv2.SimpleBlobDetector(params)
        else : 
            detector = cv2.SimpleBlobDetector_create(params)


        blobs = detector.detect(img)

        # import pdb; pdb.set_trace()

        score = 0
        ind = 0
        if len(blobs) > 0:
            for i in range(len(blobs)):
                if blobs[i].response > score:
                    score = blobs[i].response
                    ind = i

            return np.int0(blobs[i].pt)

        else:
            return None


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
