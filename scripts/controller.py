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
'''

import sys, rospy, robot_interface, rospkg
from math import pi
from random import random
from sbb_hw6.srv import *
from game_server.srv import *
from geometry_msgs.msg import Point, Quaternion, Pose
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# The controller class.
class Controller():

    # Initializes the controller as a ROS node which recieves commands.
    def __init__(self):

        # Hard-coded settings for initial pose and table location
        rospy.loginfo('Initializing controller')
        self.side = side
        self.searchD = .05
        self.tableZ = -.07
        self.scanZ = .3
        self.safeZ = .08
        self.xyTol = .01
        self.thetaTol = .3
        self.controlGain = .5
        self.pixCalZ = .15
        self.pixCalN = 78
        self.pixCalD = .044
        self.camOffset = .035
        self.xmin = .3
        self.ymin = {right: -.6, left: .1}
        self.xmax = .8
        self.ymax = {right: -.1, left: .6}

        # Define the useful "downwards" quaternion
        self.downwards = self.e2q(0, pi, 0)

        # initialize the Baxter object
        # The bowl should be within the right arm's reachable workspace on table
        rospy.loginfo('Connecting to Baxter...')
        self.baxter = robot_interface.Baxter()
        self.baxter.enable()
        rospy.loginfo('Connected to Baxter successfully.')

        # This node must send initialization info to the game server. Do it now.
        impath = rospack.get_path('sbb_hw6') + '/img/logo.png'
        img = cv2.imread(impath)
        imMsg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        self.gs = rospy.ServiceProxy('/game_server/init', Init)
        rospy.wait_for_service('/game_server/init')
        resp = self.gs('Super Baxter Bros.', imMsg)
        self.side = resp.arm

        # This node must also subscribe to the game server topic
        rospy.Subscriber('/game_server/game_state', GameState, self.storeState)

        # This node makes use of the find_stuff service
        self.cv = rospy.ServiceProxy('find_stuff', FindStuffSrv)
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
    def setW(self, side, w):
        p = self.baxter.getEndPose(side)
        p.orientation = self.e2q(0, pi, w)
        self.baxter.setEndPose(side,p)

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
        ps = self.baxter.getEndPose('right')
        height = ps.position.z - self.tableZ
        return (pix * (height/self.pixCalZ) * self.pixCalD) / self.pixCalN

    # This allows the user to teach positions using the grip buttons.
    def calibPose(self, side):
        # Define the "goalpost" positions TODO
        self.goal1 = 0
        self.goal2 = 0

    # This listens to the state publishing topic and remembers the state info internally
    def storeState(self, data):
        self.state = data.current_phase

    # Get a response from CV service: found, x, y, theta (theta doesn't matter)
    # x, y are converted to meters before returning
    def getBall(self):
        rospy.wait_for_service('find_stuff')
        ball = self.cv('ball')
	oldx = ball.x
        ball.x = self.pix2m(ball.y) - self.camOffset
        ball.y = -self.pix2m(oldx)
        return ball

    # Get a response from CV service: found, x, y, theta
    # x, y are converted to meters before returning
    def getBlock(self):
        rospy.wait_for_service('find_stuff')
        block = self.cv('block')
	oldx = block.x
        block.x = self.pix2m(block.y) - self.camOffset
        block.y = -self.pix2m(oldx)
        return block

    # Coordinates pick-place action done by Baxter
    # "from" and "to" need to be poses of the end link
    # Since we are AGAIN doing upright orientation only, neglect end effector
    def pickPlace(self, side, romP, toP):

        rospy.loginfo('Performing pick/place.')

        # Make safe pre-pick pose and go
        ps = Pose(Point(fromP.position.x, fromP.position.y, self.safeZ), fromP.orientation)
        self.baxter.openGrip(side)
        self.baxter.setEndPose(side, ps)

        # Go to pick and pick
        self.baxter.setEndPose(side, fromP)
        self.baxter.closeGrip(side)

        # Path to place pose through safe points
        self.baxter.setEndPose(side, ps)
        ps = Pose(Point(toP.position.x, toP.position.y, self.safeZ), toP.orientation)
        self.baxter.setEndPose(side, ps)
        self.baxter.setEndPose(side, toP)

        # Finish place and back off to safe height
        self.baxter.openGrip(side)
        self.baxter.setEndPose(side, ps)
        rospy.loginfo('Pick/place complete.')

    # This function attempts to put the end effector within sight of the ball centroid.
    # If the current end pose does not accomplish this, it moves to a random pose in the 
    # search plane, and checks again. Call repeatedly until succesful.
    def scanForBall(self, side):
        ball = self.getBall()
        if ball.found:
            rospy.loginfo('Ball found.')
            return ball
        else:
            xpos = self.xmin + (self.xmax - self.xmin)*random()
            ypos = self.ymin[side] + (self.ymax[side] - self.ymin[side])*random()
            ps = Pose(Point(xpos, ypos, self.scanZ), self.downwards)
            self.baxter.setEndPose(side, ps)

    # Same as scanForBall, except that the range of motion is restricted to the bowl's
    # diameter, about the initial point. Checks for exceeding xmin/xmax aren't implemented.
    def scanForBlock(self, side):
        start = self.baxter.getEndPose('right')
        while True:
            block = self.getBlock()
            if block.found:
                rospy.loginfo('Block found.')
                return block
            else:
                xpos = start.position.x + self.searchD*(random() - .5)
                ypos = start.position.y + self.searchD*(random() - .5)
                ps = Pose(Point(xpos, ypos, self.scanZ), self.downwards)
                self.baxter.setEndPose(side, ps)

    # This function must be run only when we are reliably in range of an object
    # It performs choppy closed-loop semi-proportional control of the end effector
    # until it is aligned and directly over the object.
    def controlTo(self, side, objective):

        rospy.loginfo('Controlling to %s center...' % objective)

        # Some setup: convenience function, limits, initial conditions
        clamp = lambda n, minn, maxn: max(min(maxn, n), minn)
        delY = 1000
        delX = 1000
        self.setW(0)

        # Select objective function
        if objective == 'block':
            objfun = lambda: self.getBlock()
        else:
            objfun = lambda: self.getBall()

        # Control loop. Never. Give. Up.
        while True:
            ps = self.baxter.getEndPose(side)
            obj = objfun()
            if not obj.found:
                rospy.loginfo('Lost %s! Aborting control.' % objective)
                return  
            delY = abs(obj.y)
            delX = abs(obj.x)
            if (delY < self.xyTol) and (delX < self.xyTol):
                rospy.loginfo('Center control complete.')                
                return ps
            newx = clamp(ps.position.x + (obj.x * self.controlGain), self.xmin, self.xmax)
            newy = clamp(ps.position.y + (obj.y * self.controlGain), self.ymin[side], self.ymax[side])
            ps = Pose(Point(newx, newy, ps.position.z), self.downwards)
            self.baxter.setEndPose(side, ps)

    # This does the rotational alignment control.
    def alignWith(self, side, objective):

        rospy.loginfo('Aligning with %s ...' % objective)
        delTheta = 1000

        # Select objective function
        if objective == 'block':
            objfun = lambda: self.getBlock()
        else:
            objfun = lambda: self.getBall()

        # Control loop. Never. Give. Up.
        while True:
            ps = self.baxter.getEndPose('right')
            obj = objfun()
            if not obj.found:
                rospy.loginfo('Lost %s! Aborting control.' % objective)
                return  
            if objective == 'block':
                delTheta = abs(obj.t)
            else:
                delTheta = 0
            if (delTheta < self.thetaTol):
                rospy.loginfo('Alignment complete.')                
                return ps
            newt = self.rerange(self.getW(ps)- obj.t)
            self.setW(side, newt)

    # Performs the sequence of actions required to accomplish HW5 CV objectives.
    def execute(self):
        self.scanForBowl()
        self.controlTo('bowl')
        self.scanForBlock()
        ps = self.baxter.getEndPose('right')
        ps.position.z = self.safeZ
        self.baxter.setEndPose('right', ps)
        bPose = self.controlTo('block')
        bPose = self.alignWith('block')
        bPose.position.z = self.tableZ
        self.pickPlace(bPose, self.destination)
        rospy.loginfo('All done!')

# This is what runs when the script is executed externally.
# It runs the main node function, and catches exceptions.
if __name__ == '__main__':
    if len(sys.argv) < 2:
        print "You need to specify a side!"
    try:
        controller = Controller(sys.argv[1])
        controller.execute()
    except:
        import pdb, traceback, sys
        type, value, tb = sys.exc_info()
        traceback.print_exc()
        pdb.post_mortem(tb)
