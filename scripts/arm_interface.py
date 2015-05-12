#!/usr/bin/env python

'''
    Robot Interface node for HW6.

    Since HW5, removed some fat (no more faces) and added gripper button
    access methods.
    Biggest change is it only controls a single arm - no access at all to other.
    I may also actually account for the end effector's displacement. Not now.
'''

import argparse, sys, rospy, time, threading, math
import baxter_interface, baxter_pykdl, numpy

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from std_msgs.msg import Header
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from baxter_core_msgs.msg import DigitalIOState, JointCommand
from sensor_msgs.msg import Image

# The arm class definition
# Acts as a wrapper for many useful baxter_interface methods
# Also spawns a node to interface with IK Service
class Arm():

    # Constructor
    def __init__(self, side = 'left', do_grippers = True):

        self.side = side
        
        # Create Baxter arm instance
        self.limb = baxter_interface.Limb(side)
        self.BaxEnable = baxter_interface.RobotEnable()

        # Create Baxter gripper instance
        if do_grippers:
            self.gripper = baxter_interface.Gripper(side)
            self.gripper.calibrate()

        # Create Baxter camera instance
        self.cam = baxter_interface.CameraController(side + '_hand_camera')

        # Initialize camera
        self.setCamera(side)

        # Baxter kinematics object
        self.bk = baxter_pykdl.baxter_kinematics(side)

        # Subscribe to the cuff button
        rospy.Subscriber('/robot/digital_io/' + side +'_lower_button/state', 
                         DigitalIOState, self.storeCuff)

        # We'll be controlling joints directly with messages to the joint control topic
        self.cmd = rospy.Publisher('robot/limb/' + side + '/joint_command', 
                                    JointCommand, queue_size=10)

        # Set the initial joint command message to what we have
        self.cartVel = False
        self.setJoints(self.getJoints())

        # Start up the thread that periodically commands the arm
        self.t = threading.Thread(target = self.jointPublisher)
        self.t.daemon = True
        self.t.start()

    # Enable the robot
    # Must be manually called after instantiation 
    def enable(self):
        self.BaxEnable.enable()

    # Disable the robot
    def disable(self):
        self.BaxEnable.RobotEnable.disable()

    # Check if robot is enabled
    def isEnabled(self):
        return self.BaxEnable._state.enabled
        
    # Stop the robot
    # Equivalent to hitting e-stop
    def stop(self):
        self.BaxEnable.stop()

    # Keeps track of the cuff button state
    def storeCuff(self, data):
        self.cuff = (data.state == 1)

    # Blocks until the cuff button is pressed and released
    # Can run forever...
    def waitForCuff(self):
        while True:
            if self.cuff:
                break
            time.sleep(.005)
        while True:
            if not self.cuff:
                break
            time.sleep(.005)

    # Close gripper
    # Defaults to blocking
    def closeGrip(self, block=True):
        self.gripper.close(block)

    # Open gripper
    # Defaults to blocking
    def openGrip(self, block=True):
        self.gripper.open(block)

    # Set gripper's applied force
    # Specify force in % (0-100), mapping to 0-30 N
    def setGripForce(self, limbSide, force):
        self.gripper.set_moving_force(force)
        self.gripper.set_holding_force(force)

    # Check if specified gripper is ready
    # Returns true iff gripper is calibrated, not in error state, and not moving
    def getGripReady(self, limbSide):
        return self.gripper.ready()
    
    # Method for getting joint configuration
    # Direct call to baxter_interface
    # Returns: dict({str:float})
    # unordered dict of joint name Keys to angle (rad) Values
    def getJoints(self):
        return self.limb.joint_angles()

    # Method for getting end-effector position
    def getEndPose(self):
        # Get pose as dict, the convert to Pose to return
        out = self.limb.endpoint_pose()
        return Pose(Point(*out['position']), Quaternion(*out['orientation']))

    # Method for setting joint positions
    # New as of HW6, uses the topic directly
    # This means it takes: dict({str:float}) of joints
    def setJoints(self, angles, mode = 1):
        self.mode = mode
        self.goalc = Point(*self.bk.forward_position_kinematics_arg2p(angles))
        names = [name for name in iter(angles)]
        angles = [angles[name] for name in iter(angles)]
        self.jointCmd = JointCommand(mode, angles, names)

    # Move to Z position very slowly in cartesian straight line
    def downSlowly(self, z, speed = .035, tol = .005, timeout = 10):
        err = 1000
        t = time.time()
        while (err > tol) and (time.time() -  t < timeout):
            ps = self.getEndPose()
            err = abs(ps.position.z - z)
            sign = (z - ps.position.z)/err
            self.setCartVel(0, 0, sign*speed, 0, 0, 0)
            time.sleep(.01)
        self.stopArm()

    # Directly assign velocities, as a dictionary.
    def setVelocities(self, velocities):
        self.cartVel = False
        self.setJoints(velocities, mode = 2)

    # Gets the jacobian, etc etc
    def setCartVel(self, x, y, z, wx, wy, wz):
        self.cartVel = True
        self.vels = numpy.matrix([[x],[y],[z],[wx],[wy],[wz]])

    # Sets velocities to zero!
    def stopArm(self):
        self.cartVel = False
        jd = self.getJoints()
        for key in iter(jd):
            jd[key] = 0
        self.setVelocities(jd)

    def jointPublisher(self):
        while True:
            t = time.time()
            if self.cartVel:
                jac = self.bk.jacobian_pseudo_inverse()
                res = jac*self.vels
                vels = [a[0] for a in res.tolist()]
                jd = self.getJoints()
                for n, j in enumerate(['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']):
                    jd[self.side + '_' + j] = vels[n]
                self.setJoints(jd, mode = 2)
            self.cmd.publish(self.jointCmd)
            while time.time() - t < .01:
                pass

    # Method for calculating joint angles given a desired end-effector pose
    def getIKGripper(self, setPose):
        # Prepare the request for Baxter's IK
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ps = PoseStamped(header=hdr, pose=setPose,)
        ikreq = SolvePositionIKRequest([ps], [], 0)

        # Make the service call
        srvName = 'ExternalTools/' + self.side + '/PositionKinematicsNode/IKService'
        srvAlias = rospy.ServiceProxy(srvName, SolvePositionIK)
        rospy.wait_for_service(srvName)
        resp = srvAlias(ikreq)

        # Get IK response and convert to joint position dict
        if (resp.isValid[0]):
            return dict(zip(resp.joints[0].name, resp.joints[0].position))
        else:
            return None

    # Methods for setting cartesian position of hand
    # setPose is a global pose
    def setEndPose(self, setPose):
        self.cartVel = False
        ik_joints = self.getIKGripper(setPose)
        if ik_joints is None:
            rospy.loginfo('Bad pose commanded - ignoring.')
        else:
            self.setJoints(ik_joints, 1)
            self.waitForPose()

    def waitForPose(self, tol = .03, timeout = 5):
        t = time.time()
        while (self.getCartErr() > tol) and (time.time() - t < timeout):
            time.sleep(.01)

    def getCartErr(self):
        cur = self.getEndPose()
        a = math.pow(self.goalc.x - cur.position.x, 2)
        b = math.pow(self.goalc.y - cur.position.y, 2)
        c = math.pow(self.goalc.z - cur.position.z, 2)
        return math.sqrt(a + b + c)

    # Camera Settings
    def setCamera(self, name, res=(640,400), fps=10):
        self.cam.open()
        self.cam.resolution = res
        self.cam.fps = fps            
