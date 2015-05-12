#!/usr/bin/env python

'''
    Controller node for HW6.

    The controller assumes one of several states:

    Initialize
        Initialize all objects, send logo message to Baxter, etc.
        Poll for 3 grip buttons: one at circle, one at midfield, one on stack.

    Setup
        Take the three blocks from stack and lay them out as a wall.

    Defend
        When ball not in range, swing hand back and forth across goal.

    Acquire
        When ball in range, scan for it and pick it up with CV.

    Toss
        If we have the ball, execute toss maneuvre and go back to Defend.

    These are different from phases, which are retrieved from the game server.
'''

import sys, time, copy, rospy, arm_interface, rospkg, cv2, cv_bridge
from math import pi, atan
from random import random
from sbb_hw6.srv import *
from game_server.srv import *
from game_server.msg import *
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Quaternion, Pose
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# The controller class.
class Controller():

    # Initializes the controller as a ROS node which recieves commands.
    def __init__(self):

        # Launch dat node
        rospy.init_node("SBB_Game_Node", anonymous = True)

        # This node must send initialization info to the game server. Do it now.
        rospy.loginfo('Initializing controller')
        rospy.loginfo('Connecting to game server...')
        rospack = rospkg.RosPack()
        impath = rospack.get_path('sbb_hw6') + '/img/logo.png'
        img = cv2.imread(impath)
        imMsg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        self.gs = rospy.ServiceProxy('/game_server/init', Init)
        rospy.wait_for_service('/game_server/init')
        resp = self.gs('Super Baxter Bros.', imMsg)
        self.side = resp.arm

        # Let our other nodes what side we're using
        rospy.set_param('sbb_side', self.side)
        rospy.loginfo('Assigned side ' + self.side + ' from game server')

        # This is a useful sign to have later
        if self.side == 'left':
            self.ys = -1
        else:
            self.ys = 1

        # Get the number of blocks from a parameter
        self.nBlocks = rospy.get_param('/num_blocks')

        # This node must also subscribe to the game server topic
        rospy.Subscriber('/game_server/game_state', GameState, self.storeState)

        # The posession tracker also publishes to a topic
        rospy.Subscriber('sbb/ball_side', Bool, self.storeSide)
        self.haveBall = True

        # This node makes use of the find_stuff service
        self.cv = rospy.ServiceProxy('find_stuff', FindStuffSrv)

        # Initialize our arm
        rospy.loginfo('Connecting to Baxter...')
        self.arm = arm_interface.Arm(self.side)
        self.arm.enable()
        self.arm.stopArm()

        # Hard-coded settings for initial pose and table location
        self.swingUp = .5
        self.swingDown = .05
        self.blockSpacing = .05
        self.scanD = .25
        self.controlD = .1
        self.xyTol = .01
        self.controlGain = 1
        self.controlMax = .05
        self.pixCalZ = .15
        self.pixCalN = 78
        self.pixCalD = .044
        self.camOffset = .027
        self.xmin = .3
        self.xmax = .8
        self.ymin = {'right': -.6, 'left': .1}
        self.ymax = {'right': -.1, 'left': .6}
        self.ymax = self.ymax[self.side]
        self.ymin = self.ymin[self.side]

        # Hard-coded joint positions for toss maneuver
        self.tossJ = {self.side + '_w0': -self.ys*pi/2,  #square 
                      self.side + '_w1': pi/2,  #square 
                      self.side + '_w2': -self.ys*1.4, #bent
                      self.side + '_e0': self.ys*pi/2, #square
                      self.side + '_e1': 1.2, #bent
                      self.side + '_s0': -self.ys*0.8, 
                      #controlled to - direction for left, + for right
                      self.side + '_s1': 0} #square
        self.tossV = copy.deepcopy(self.tossJ)
        for i in iter(self.tossV):
            self.tossV[i] = 0
        self.tossV[self.side + '_s0'] = self.ys*10

        self.phase = 0

        # Define the useful "downwards" quaternion
        self.downwards = self.e2q(0, pi, 0)

        rospy.loginfo('Initialization successful.')

    # Convenience function for euler_to_quaternion that returns as Quaternion.
    def e2q(self, x, y, z):
        q = tuple(quaternion_from_euler(x, y, z))
        return Quaternion(*q)

    def setZDown(self, z):
        ps = self.arm.getEndPose()
        ps.position.z = z
        ps.orientation = self.downwards
        self.arm.setEndPose(ps)

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
        self.goal = [None, None]
        rospy.loginfo('Place arm over left goalpost and press cuff...')
        self.arm.waitForCuff()
        self.goal[0] = self.arm.getEndPose()

        rospy.loginfo('Place arm over right goalpost and press cuff...')
        self.arm.waitForCuff()
        self.goal[1] = self.arm.getEndPose()

        rospy.loginfo('Place arm over top of stack and press cuff...')
        self.arm.waitForCuff()
        self.stackTop = self.arm.getEndPose()

        rospy.loginfo('Place arm over field center and press cuff...')
        self.arm.waitForCuff()
        self.center = self.arm.getEndPose()

        self.tableZ = self.center.position.z
        self.controlZ = self.tableZ + self.controlD
        self.safeZ = self.tableZ + self.nBlocks*.044
        self.scanZ = self.tableZ + self.scanD

        # Pre-calculate the x/y spacing of blocks
        self.blockY = self.center.position.y - self.ys*.2
        width = (self.nBlocks-1)*self.blockSpacing
        self.blockX = range(self.nBlocks)
        for i, x in enumerate(self.blockX):
            self.blockX[i] = x*self.blockSpacing + self.center.position.x - width/2

    # This listens to the state publishing topic and remembers the state info internally
    def storeState(self, data):
        self.phase = data.current_phase

    # This listens to the side tracking topic and keeps it for us
    def storeSide(self, data):
        self.haveBall = data.data

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
        ps = Pose(Point(fromP.position.x, fromP.position.y, fromP.position.z + .08), fromP.orientation)
        self.arm.openGrip()
        self.arm.setEndPose(ps)

        # Go to pick and pick
        self.arm.downSlowly(fromP.position.z)
        self.arm.closeGrip()

        # Path to place pose through safe points
        self.arm.setEndPose(ps)
        ps = Pose(Point(toP.position.x, toP.position.y, toP.position.z + .08), toP.orientation)
        self.arm.setEndPose(ps)
        self.arm.downSlowly(toP.position.z)

        # Finish place and back off to safe height
        self.arm.openGrip()
        self.arm.setEndPose(ps)
        rospy.loginfo('Pick/place complete.')

    # This function attempts to put the end effector within sight of the ball centroid.
    # If the current end pose does not accomplish this, it moves to a random pose in the 
    # search plane, and checks again. Call repeatedly until succesful.
    def scanForBall(self):
        self.setZDown(self.scanZ)
        while self.haveBall:
            time.sleep(.5)
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
        rospy.loginfo('Controlling to ball...')

        # Some setup: convenience function, limits, initial conditions
        delY = 1000
        delX = 1000

        # Control loop. Never. Give. Up.
        while self.haveBall:
            obj = self.getBall()
            if not obj.found:
                rospy.loginfo('Lost ball! Aborting control.')
                self.arm.stopArm()
                return  False
            delY = abs(obj.y)
            delX = abs(obj.x)
            if (delY < self.xyTol) and (delX < self.xyTol):
                rospy.loginfo('Center control complete.')
                self.arm.stopArm()                
                return True
            vx = min([self.controlMax, (obj.x * self.controlGain)])
            vy = min([self.controlMax, (obj.y * self.controlGain)])
            self.arm.setCartVel(vx, vy, 0, 0, 0, 0)
            time.sleep(.02)

    def acquireBall(self):
        self.arm.openGrip()
        step = 1
        while step > 0:
            if not self.haveBall:
                return False
            if step == 1:
                self.setZDown(self.scanZ)
                done = self.scanForBall()
                step = 2 if done else 1
            elif step == 2:
                self.setZDown(self.scanZ)
                done = self.controlToBall()
                step = 3 if done else 1
            elif step == 3:
                self.setZDown(self.controlZ)
                done = self.controlToBall()
                step = 0 if done else 1
        self.arm.downSlowly(self.tableZ, speed = .05)
        self.arm.closeGrip()
        self.setZDown(self.scanZ)
        return True

    def launchBall(self):
        self.arm.setJoints(self.tossJ)
        self.arm.waitForPose()
        time.sleep(2)
        self.arm.setVelocities(self.tossV)
        time.sleep(self.swingUp)
        self.arm.openGrip()
        time.sleep(self.swingDown)
        self.arm.stopArm()

    def unstackBlocks(self):
        # Unstack the blocks onto the table
        for bl in range(self.nBlocks):
            fromP = copy.deepcopy(self.stackTop)
            fromP.position.z = fromP.position.z - bl*.044
            toP = Pose(Point(self.blockX[bl], self.blockY, self.tableZ + .022), self.downwards)
            self.pickPlace(fromP, toP)

    # Performs the sequence of actions required to accomplish HW6 objectives.
    def playGame(self):

        # Wait until the game starts
        rospy.loginfo('Waiting for game to start...')
        while True:
            if self.phase > 0:
                break
            time.sleep(.005)

        # First, calibrate some locations on the field.
        self.calibPose()

        # Wait until the game phase is 2 (block stacking)
        rospy.loginfo('Calibration done. Waiting for next game phase...')
        while True:
            if self.phase > 1:
                break
            time.sleep(.005)
        self.unstackBlocks()

        # Wait until the game phase is 3 (main game)
        rospy.loginfo('Stacking done. Waiting for next game phase...')
        rospy.loginfo('(Put on our grippers during this time!)')
        self.arm.stopArm()
        while True:
            if self.phase > 2:
                break
            time.sleep(.005)

        # Enter the main state loop
        curGoal = 0
        while self.phase == 3:

            if not self.haveBall:          # Goalkeep
                curGoal = (curGoal + 1) % 2
                self.arm.setEndPose(self.goal[curGoal])

            else:        # Pick ball
                rospy.loginfo('The ball is on our side...')
                res = self.acquireBall()
                if res:
                    self.launchBall()

# This is what runs when the script is executed externally.
# It runs the main node function, and catches exceptions.
if __name__ == '__main__':
    try:
        controller = Controller()
        controller.playGame()
    except:
        pass
