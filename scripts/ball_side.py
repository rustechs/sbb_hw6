import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

class ballSide:

	def __init__(self,side):
		# Subscribe to head camera frame topic
	    self.frame_sub = rospy.Subscriber('usb_cam_head/image_compressed',Image,self.parseFrame)
	    # Publish parsed frames to debug topic
	    self.frame_pub = rospy.Publisher('sbb/cv/head_cam_overlay',Image,queue_size=10)
	    # Publish parsing to topic for controller
	    self.ball_pub = rospy.Publisher('sbb/ball_side',Bool,queue_size=10)

	    # Create bridge to convert between CV and ROS images
	    self.br = CvBridge()

	    # Set what side we're on
	    self.side = side

	    # Set side mask percentage
	    self.maskSide = 0.45	

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
	    self.center = (int(self.w/2),int(self.h/2))

	   	# Equalize histogram of incoming image
	    self.img = cv2.equalizeHist(self.img)

	    # Convert image to HSV
	    imgHSV = cv2.cvtColor(self.img,cv2.COLOR_BGR2HSV)

	    # Mask off opponent side
	    if self.side == 'left':
	    	imgHSV = imgHSV[:,]
	    elif:
	    	imgHSV = imgHSV[:,]


	    # Publish result to topic
	    self.ball_pub.publish(ourSide)

	    # publish debug overlay image
	    self.pubDebugImg()

	def pubDebugImg(self):
		self.frame_pub.publish(self.br.cv2_to_imgmsg(self.img, "bgr8"))

def main():

	# Init node
    rospy.init_node('ball_side', anonymous = True)

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
