#!/usr/bin/env python

'''
    Robot Interface node for HW6.

    Since HW5, removed some fat (no more faces) and added gripper button
    access methods.
    Biggest change is it only controls a single arm - no access at all to other.
    I may also actually account for the end effector's displacement. Not now.
'''

import argparse, sys, rospy, baxter_interface, sched, time

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

        rospy.init_node("SBB_Arm_Node", anonymous = True)
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

        # Subscribe to the cuff button
        rospy.Subscriber('/robot/digital_io/' + side +'_lower_button/state', 
                         DigitalIOState, self.storeCuff)

        # We'll be controlling joints directly with messages to the joint control topic
        self.cmd = rospy.Publisher('robot/limb/' + side + '/joint_command', JointCommand)

        # Set the initial joint command message to what we have
        self.setJoints(self.getJoints)

        # Start up the scheduler that periodically commands the arm
        self.s = sched.scheduler(time.time, time.sleep)
        self.s.enter(.1, 1, self.jointPublisher, ())

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
        # Get pose as dict
        out = self.limb.endpoint_pose() 

        # From dict to Pose object
        return Pose(Point(*out['position']), Quaternion(*out['orientation']))

    # Method for setting joint positions
    # New as of HW6, uses the topic directly
    # This means it takes: dict({str:float}) of joints
    def setJoints(self, angles):
        names = [name for name in iter(angles)]
        angles = [j[name] for name in iter(angles)]
        self.jointCmd = JointCommand(1, angles, names)

    def jointPublisher(self):
        self.cmd.publish(self.jointCmd)
        self.s.enter(.1, 1, self.jointPublisher, ())

    # Method for calculating joint angles given a desired end-effector pose
    def getIKGripper(self, setPose):

        # Prepare the request
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
            print("IK service: INVALID POSE - No Valid Joint Solution Found.")

    # Methods for setting cartesian position of hand
    # setPose is a global pose
    def setEndPose(self, setPose):
        ik_joints = self.getIKGripper(setPose)
        self.setJoints(ik_joints)

    # This  particular method only changes x, y, z of pose
    # Use by naming arguments you want to change
    def setEndXYZO(self, x = None, y = None, z = None, o = None):
        ps = self.getEndPose(limbSide)
        if x is None:
            x = ps.position.x
        if y is None:
            y = ps.position.y
        if z is None:
            z = ps.position.z
        if o is None:
            o = ps.orientation
        self.setEndPose(Point(x, y, z), o)

    # And this changes orientation only. Takes a quaternion.
    def setEndOrientation(self, o):
        ps = self.getEndPose()
        ps.orientation = o
        self.setEndPose(ps)

    # Camera Settings
    def setCamera(self, name, res=(640,400), fps=10):
        self.cam.open()
        self.cam.resolution = res
        self.cam.fps = fps            
