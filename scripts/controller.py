#!/usr/bin/env python

'''
    Controller node for HW6.

    The controller assumes one of three states:

    Initialize (State 0)
        Initialize all objects, send logo message to Baxter, etc.
        Poll for 3 grip buttons: one at circle, one at midfield, one on stack.

    Setup (State 1)
        Take the three blocks from stack and lay them out as a wall.

    Defend (State 2)
        When ball not in range, swing hand back and forth across goal.

    Acquire (State 3)
        When ball in range, scan for it and pick it up with CV.

    Toss (State 4)
        If we have the ball, execute toss maneuvre and go back to Defend.

    These are different from phases, which are retrieved from the game server.
'''

import sys, rospy, arm_interface, rospkg
from math import pi
from random import random
from time import sleep
from sbb_hw6.srv import *
from game_server.srv import *
from geometry_msgs.msg import Point, Quaternion, Pose
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# The controller class.
class Controller():

    # Initializes the controller as a ROS node which recieves commands.
    def __init__(self):

        # This node must send initialization info to the game server. Do it now.
        rospy.loginfo('Initializing controller')
        rospy.loginfo('Connecting to game server...')
        impath = rospack.get_path('sbb_hw6') + '/img/logo.png'
        img = cv2.imread(impath)
        imMsg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        self.gs = rospy.ServiceProxy('/game_server/init', Init)
        rospy.wait_for_service('/game_server/init')
        resp = self.gs('Super Baxter Bros.', imMsg)
        self.side = resp.arm

        # Let our other nodes what side we're using
        rospy.set_param('sbb_side', self.side)

        # This is a useful sign to have later
        if self.side == 'left':
            self.ys = -1
        else:
            self.ys = 1

        # Get the number of blocks from a parameter
        self.nBlocks = rospy.get_param('/num blocks')

        # This node must also subscribe to the game server topic
        rospy.Subscriber('/game_server/game_state', GameState, self.storeState)

        # The posession tracker also publishes to a topic
        rospy.Subscriber('sbb/ball_side', Bool, self.storeSide)

        # This node makes use of the find_stuff service
        self.cv = rospy.ServiceProxy('find_stuff', FindStuffSrv)

        # Initialize our arm
        rospy.loginfo('Connecting to Baxter...')
        self.arm = arm_interface.Arm(self.side)
        self.arm.enable()

        # Hard-coded settings for initial pose and table location
        self.blockSpacing = .1
        self.searchD = .05
        self.tableZ = -.07
        self.scanZ = .3
        self.safeZ = self.tableZ + self.nBlocks*.044
        self.xyTol = .01
        self.thetaTol = .3
        self.controlGain = .5
        self.pixCalZ = .15
        self.pixCalN = 78
        self.pixCalD = .044
        self.camOffset = .035
        self.xmin = .3
        self.ymin = {right: -.6, left: .1}
        self.ymin = self.ymin[self.side]
        self.xmax = .8
        self.ymax = {right: -.1, left: .6}
        self.ymax = self.ymax[self.side]

        self.phase = 0
        self.state = 0

        # Define the useful "downwards" quaternion
        self.downwards = self.e2q(0, pi, 0)

        rospy.loginfo('Initialization successful.')

    # Convenience function for euler_to_quaternion that returns as Quaternion.
    def e2q(self, x, y, z):
        q = tuple(quaternion_from_euler(x, y, z))
        return Quaternion(*q)

    # Convenience function for getting the angular alignment in YZ plane from pose
    def getW(self, p):
        eu = euler_from_quaternion([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return self.rerange(eu[2] + pi)

    # Convenience function for changing the angular alignment in YZ only
    def setW(self, w):
        p = self.arm.getEndPose()
        p.orientation = self.e2q(0, pi, w)
        self.arm.setEndPose(p)

    # Puts a value in the -pi to pi range
    def rerange(self, val):
        if val > pi:
            return (val % (2*pi)) - 2*pi
        elif val < -pi:
            return -(-val % (2*pi)) + 2*pi
        else:
            return val

    # This function converts pixels to meters on the table plan given a height.
    # It assumes/requires a downward orientation.
    # It takes care of checking the height off the table itself.
    def pix2m(self, pix):
        ps = self.arm.getEndPose()
        height = ps.position.z - self.tableZ
        return (pix * (height/self.pixCalZ) * self.pixCalD) / self.pixCalN

    # This allows the user to teach positions using the grip buttons.
    def calibPose(self):

        # Get some positions from the user
        goal = [None, None]
        rospy.loginfo('Place arm over left goalpost and press cuff...')
        self.arm.waitForCuff()
        self.goal[0] = self.arm.getEndPose()

        rospy.loginfo('Place arm over right goalpost and press cuff...')\
        self.arm.waitForCuff()
        self.goal[1] = self.arm.getEndPose()

        rospy.loginfo('Place arm over top of stack and press cuff...')\
        self.arm.waitForCuff()
        self.stackTop = self.arm.getEndPose()

        rospy.loginfo('Place arm over field center and press cuff...')\
        self.arm.waitForCuff()
        self.center = self.arm.getEndPose()

        # Pre-calculate the x/y spacing of blocks
        self.blockY = self.center.position.y + self.ys*.2
        width = (self.nBlocks-1)*self.blockSpacing
        self.blockX = range(self.bBlocks)
        for x in self.blockX:
            x = x*self.blockSpacing + self.center.position.x - width/2

    # This listens to the state publishing topic and remembers the state info internally
    def storeState(self, data):
        self.phase = data.current_phase

    # This listens to the side tracking topic and keeps it for us
    def storeSide(self, data):
        self.side = data.data

    # Get a response from CV service: found, x, y, theta (theta doesn't matter)
    # x, y are converted to meters before returning
    def getBall(self):
        rospy.wait_for_service('find_stuff')
        ball = self.cv('ball')
	oldx = ball.x
        ball.x = self.pix2m(ball.y) - self.camOffset
        ball.y = -self.pix2m(oldx)
        return ball

    # Coordinates pick-place action done by Baxter
    # "from" and "to" need to be poses of the end link
    # Since we are AGAIN doing upright orientation only, neglect end effector
    def pickPlace(self, fromP, toP):

        rospy.loginfo('Performing pick/place.')

        # Make safe pre-pick pose and go
        ps = Pose(Point(fromP.position.x, fromP.position.y, self.safeZ), fromP.orientation)
        self.arm.openGrip()
        self.arm.setEndPose(ps)

        # Go to pick and pick
        self.arm.setEndPose(fromP)
        self.arm.closeGrip()

        # Path to place pose through safe points
        self.arm.setEndPose(ps)
        ps = Pose(Point(toP.position.x, toP.position.y, self.safeZ), toP.orientation)
        self.arm.setEndPose(ps)
        self.arm.setEndPose(toP)

        # Finish place and back off to safe height
        self.arm.openGrip()
        self.arm.setEndPose(ps)
        rospy.loginfo('Pick/place complete.')

    # This function attempts to put the end effector within sight of the ball centroid.
    # If the current end pose does not accomplish this, it moves to a random pose in the 
    # search plane, and checks again. Call repeatedly until succesful.
    def scanForBall(self):
        while self.state == 3:
            ball = self.getBall()
            if ball.found:
                rospy.loginfo('Ball found.')
                return True
            else:
                xpos = self.xmin + (self.xmax - self.xmin)*random()
                ypos = self.ymin + (self.ymax - self.ymin)*random()
                ps = Pose(Point(xpos, ypos, self.scanZ), self.downwards)
                self.arm.setEndPose(ps)
        return False

    # This function must be run only when we are reliably in range of an object
    # It performs choppy closed-loop semi-proportional control of the end effector
    # until it is aligned and directly over the object.
    def controlToBall(self):

        rospy.loginfo('Controlling to %s center...' % objective)

        # Some setup: convenience function, limits, initial conditions
        clamp = lambda n, minn, maxn: max(min(maxn, n), minn)
        delY = 1000
        delX = 1000
        self.setW(0)

        # Control loop. Never. Give. Up.
        while self.state == 3:
            ps = self.arm.getEndPose()
            obj = self.getBall()
            if not obj.found:
                rospy.loginfo('Lost %s! Aborting control.' % objective)
                return  False
            delY = abs(obj.y)
            delX = abs(obj.x)
            if (delY < self.xyTol) and (delX < self.xyTol):
                rospy.loginfo('Center control complete.')                
                return True
            newx = clamp(ps.position.x + (obj.x * self.controlGain), self.xmin, self.xmax)
            newy = clamp(ps.position.y + (obj.y * self.controlGain), self.ymin, self.ymax)
            ps = Pose(Point(newx, newy, ps.position.z), self.downwards)
            self.arm.setEndPose(ps)

    # Performs the sequence of actions required to accomplish HW6 objectives.
    def playGame(self):

        # Wait until the game starts
        rospy.loginfo('Waiting for game to start...')
        while 1
            if self.phase > 0
                break
            sleep(.005)

        # First, calibrate some locations on the field.
        self.calibPose()

        # Wait until the game phase is 2 (block stacking)
        rospy.loginfo('Calibration done. Waiting for next game phase...')
        while 1
            if self.phase > 1
                break
            sleep(.005)

        # Unstack the blocks onto the table
        for bl in range(self.nBlocks):
            fromP = self.stackTop
            fromP.position.z = fromP.position.z - bl*.044
            toP = Pose(Point(self.blockX[bl], self.blockY, self.tableZ), self.downwards)
            self.pickPlace(fromP, toP)

        # Wait until the game phase is 3 (main game)
        rospy.loginfo('Stacking done. Waiting for next game phase...')
        while 1
            if self.phase > 2
                break
            sleep(.005)

        # Enter the main state loop
        curGoal = 0
        while self.phase > 0

            posession = 0 #todo getPosession, set state

            if state == 2:          # Goalkeep
                curGoal = (curGoal + 1) % 1
                self.arm.setEndPose(self.goal[curGoal])

            elif state == 3:        # Pick ball
                success = scanForBall()
                if success
                    success = controlToBall()
                    if success
                        self.arm.closeGrip()

            elif state == 4:        # Launch ball
                # Launch ball
                pass

            else
                rospy.loginfo('Error: Lost state!')

# This is what runs when the script is executed externally.
# It runs the main node function, and catches exceptions.
if __name__ == '__main__':
    try:
        controller = Controller()
        controller.playGame()
    except:
        import pdb, traceback, sys
        type, value, tb = sys.exc_info()
        traceback.print_exc()
        pdb.post_mortem(tb)
