#!/usr/bin/env python

'''
Generates a smooth trajectory from the joint's current position to
its desired position.

See: https://humancenteredrobotics.atlassian.net/wiki/display/WBC/2015.02.19+A+Simple+Smooth+Time-Based+Joint+Trajectory+Generator
'''

import sys, getopt     # for getting and parsing command line arguments and exiting
import time
import math
import threading
import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from controlit_core.srv import get_parameters

def radToDeg(rad):
    return rad / 3.14 * 180

class SimpleSmoothJointTrajectoryGenerator:
    def __init__(self, jointPosTaskName, pFinal, vDesired, aDesired, jointIndex, updateFreq):
        """
        The constructor.

        Keyword arguments:
        jointPosTaskName -- The name of the joint position task.
        pFinal -- The final position of the joint
        aDesired -- The desired acceleration.
        tFactor -- The time factor.
        jointIndex -- The index of the joint being controlled.
        updateFreq -- The frequency at which the reference position should be sent
        """

        self.jointPosTaskName = jointPosTaskName
        self.xFinal = pFinal
        self.vDesired = vDesired
        self.aDesired = aDesired
        self.jointIndex = jointIndex
        self.updateFreq = updateFreq
        self.updatePeriod = 1.0 / updateFreq

    def init(self):

        # Verify parameters are valid
        if self.aDesired == 0:
            print "init: ERROR: Invalid desired acceleration. Value must not be zero."
            return False

        if self.vDesired == 0:
            print "init: ERROR: Invalid desired velocity. Value must not be zero."
            return False

        # Get the number of joints
        self.numJoints = -1
        self.jointName = None

        rospy.wait_for_service('diagnostics/getRealJointIndices')
        try:
            getJointSrv = rospy.ServiceProxy("diagnostics/getRealJointIndices", get_parameters)
            resp = getJointSrv()
            # print "Got Response {0}".format(resp)
            self.numJoints = len(resp.params.values)
            self.jointName = resp.params.values[jointIndex].value
            print "init: Automatically obtained parameters:\n"\
                   "  - Number of joints: {0}\n"\
                   "  - Name of joint being moved: {1}".format(
                self.numJoints, self.jointName)
        except rospy.ServiceException, e:
            print "init: Failed to call service: {0}".format(e)
            rospy.signal_shutdown("done")
            return False

        if self.jointIndex >= self.numJoints:
            print "init: ERROR: Invalid joint index of {0}. Number of joints is {1}.".format(
                self.jointIndex, self.numJoints)
            return False

        # Get the joint's initial position
        self.currentPosture = None
        self.postureTaskActualSubscriber = rospy.Subscriber(
            self.jointPosTaskName + "/actualPosition", Float64MultiArray, self.jointTaskActualCallback)

        pauseCount = 0
        printWarning = False
        while not rospy.is_shutdown() and self.currentPosture == None:

            if printWarning:
                print "init: Waiting to receive actual joint positions..."
                    
            time.sleep(0.5)
            pauseCount = pauseCount + 1
            if pauseCount > 5 and not printWarning:
                printWarning = True

        if rospy.is_shutdown():
            return True

        self.xInit = self.currentPosture[self.jointIndex]
        print "init: Current position of {0} is {1} radians ({2} degrees)...".format(self.jointName, self.xInit, radToDeg(self.xInit))

        if self.xInit > self.xFinal:
            self.vDesired = -1 * abs(self.vDesired)
            self.aDesired = -1 * abs(self.aDesired)
        else:
            self.vDesired = abs(self.vDesired)
            self.aDesired = abs(self.aDesired)
        
        print "init: After adjusting for direction of movement:\n"\
              "  - desired velocity = {0}\n"\
              "  - desired acceleration = {1}".format(
                self.vDesired, self.aDesired)

        # Create the goal message. Store the current joint states into the messages
        dimPos = MultiArrayDimension()
        dimPos.size = self.numJoints
        dimPos.label = "goalPosMsg"
        dimPos.stride = 1

        self.goalPosMsg = Float64MultiArray()
        for ii in range(0, self.numJoints):
            self.goalPosMsg.data.append(self.currentPosture[ii])  # assumes order matches, which it should since the current posture was published by the same task
        self.goalPosMsg.layout.dim.append(dimPos)
        self.goalPosMsg.layout.data_offset = 0

        dimVel = MultiArrayDimension()
        dimVel.size = self.numJoints
        dimVel.label = "goalVelMsg"
        dimVel.stride = 1

        self.goalVelMsg = Float64MultiArray()
        for ii in range(0, self.numJoints):
            self.goalVelMsg.data.append(0)  # initial velocity is zero
        self.goalVelMsg.layout.dim.append(dimVel)
        self.goalVelMsg.layout.data_offset = 0

        self.tAccel = self.vDesired / self.aDesired
        self.dAccel = self.tAccel * self.vDesired / 2   # distance during acceleration

        totalDist = self.xFinal - self.xInit
        if abs(self.dAccel * 2) > abs(totalDist):

            # tMiddle = math.sqrt((self.xFinal / 2 - self.xInit) * 2 / self.aDesired)
            tMiddle = math.sqrt((self.xFinal - self.xInit) / self.aDesired)
            vMax = self.aDesired * tMiddle

            print "init: WARNING: Desired acceleration prevents desired velocity from being reached!\n"\
                  "  - desired velocity: {0}\n"\
                  "  - acceleration and deceleration distances: {1} (double this if the desired velocity is actually reached)\n"\
                  "  - total distance needed: {2}\n"\
                  "  - max velocity possible: {3}\n"\
                  "  - max velocity time: {4}".format(
                    self.vDesired, self.dAccel, totalDist, vMax, tMiddle)

            self.vDesired = vMax   # set the new max velocity
            self.tAccel = tMiddle
            self.dAccel = (self.xFinal - self.xInit) / 2.0
            self.tSteady = 0
        else:
            self.tSteady = (totalDist - 2.0 * self.dAccel) / self.vDesired

        print "init: trajectory parameters:\n"\
              "  - T_accel = {0} seconds\n"\
              "  - D_accel = {1} radians\n"\
              "  - total distance = {2} radians\n"\
              "  - T_steady = {3}".format(self.tAccel, self.dAccel, totalDist, self.tSteady)


        return True

    def jointTaskActualCallback(self, msg):
        """
        The callback method for the subscription to the joint task's actual joint positions.

        Keyword arguments:
        msg -- The message received.
        """

        self.currentPosture = msg.data

    def start(self):
        """
        Starts the publishing of the trajectory.
        """

        # Create the publisher and ensure connection is valid
        goalPosPublisher = rospy.Publisher(self.jointPosTaskName + "/goalPosition", Float64MultiArray, queue_size=1)
        goalVelPublisher = rospy.Publisher(self.jointPosTaskName + "/goalVelocity", Float64MultiArray, queue_size=1)
        
        pauseCount = 0
        printWarning = False
        while not rospy.is_shutdown() and goalPosPublisher.get_num_connections() == 0 \
            and goalVelPublisher.get_num_connections() == 0:

            if printWarning:
                if goalPosPublisher.get_num_connections() == 0:
                    print "Waiting on goal position subscriber..."
                if goalVelPublisher.get_num_connections() == 0:
                    print "Waiting on goal velocity subscriber..."
                    
            time.sleep(0.5)
            pauseCount = pauseCount + 1
            if pauseCount > 5 and not printWarning:
                printWarning = True

        if rospy.is_shutdown():
            return

        # Compute the times that denote the phases
        t0 = rospy.get_rostime()
        t0_sec = t0.to_sec()
        t1 = self.tAccel          # relative to t0
        t2 = t1 + self.tSteady    # relative to t0
        t3 = t2 + self.tAccel     # relative to t0

        # Compute some other useful parameters
        accel = self.vDesired / self.tAccel
        c1    = self.vDesired + accel * t2
        c2    = accel / 2 * math.pow(t1, 2) + self.xInit - self.vDesired * t1
        c3    = self.vDesired * t2 + c2 + accel / 2 * math.pow(t2, 2) - c1 * t2

        print "start: Trajectory time parameters:\n"\
              "  - t0 = 0 ({0} absolute) seconds\n"\
              "  - t1 = {1} seconds\n"\
              "  - t2 = {2} seconds\n"\
              "  - t3 = {3} seconds\n"\
              "  - accel = {4} radians / second^2\n"\
              "  - c1 = {5}\n"\
              "  - c2 = {6}\n"\
              "  - c3 = {7}".format(t0_sec, t1, t2, t3, accel, c1, c2, c3)

        doneTraj = False
        while not rospy.is_shutdown() and not doneTraj:

            deltaTime = (rospy.get_rostime() - t0).to_sec()

            goalVel = 0

            if deltaTime <= t1:
                goalPos = accel / 2 * math.pow(deltaTime, 2) + self.xInit
                goalVel = accel * deltaTime
            elif deltaTime <= t2:
                goalPos = self.vDesired * deltaTime + c2
                goalVel = self.vDesired
            elif deltaTime <= t3:
                goalPos = -1 * accel / 2 * math.pow(deltaTime, 2) + c1 * deltaTime + c3
                goalVel = -1 * accel * deltaTime + c1
            else:
                goalPos = self.xFinal
                goalVel = 0
                doneTraj = True

            self.goalPosMsg.data[self.jointIndex] = goalPos
            goalPosPublisher.publish(self.goalPosMsg)

            self.goalVelMsg.data[self.jointIndex] = goalVel
            goalVelPublisher.publish(self.goalVelMsg)

            if not doneTraj:
                rospy.sleep(self.updatePeriod)


# Main method
if __name__ == "__main__":

    rospy.init_node('SimpleSmoothJointTrajectoryGenerator', anonymous=True)

    # Define default values for the command line arguments
    jointPosTaskName = 'JointPositionTask'
    
    pFinal = 0
    vDesired = 0.1
    aDesired = 0.5

    jointIndex = 0

    updateFreq = 100

    usageStr = "Usage: python SimpleSmoothJointTrajectoryGenerator.py [parameters]\n"\
               "Valid parameters include:\n"\
               " -t or --jointPosTaskName [name of joint position task] (default {0})\n"\
               " -p or --pFinal [final joint position] (default {1})\n"\
               " -v or --vDesired [the desired steady-state velocity] (default {2})\n"\
               " -a or --aDesired [the desired acceleration] (default {3})\n"\
               " -j or --jointIndex [joint index], the index of the joint to move in a sine wave (default {4}\n"\
               " -f or --updateFreq [update frequency, the rate at which new points along the trajectory are published in Hz] (default {5})".format(
        jointPosTaskName, pFinal, vDesired, aDesired, jointIndex, updateFreq)

    # Parse the command line arguments
    try:
        opts, args = getopt.getopt(sys.argv[1:],"ht:p:v:a:j:f:",["jointPosTaskName=", "pFinal=", 
            "vDesired=", "aDesired=", "jointIndex=", "updateFreq="])
    except getopt.GetoptError:
       rospy.logerr(usageStr)
       sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            print usageStr
            sys.exit()
        elif opt in ("-t", "--jointPosTaskName"):
            jointPosTaskName = arg
        elif opt in ("-p", "--pFinal"):
            pFinal = float(arg)
        elif opt in ("-v", "--vDesired"):
            vDesired = float(arg)
        elif opt in ("-a", "--aDesired"):
            aDesired = float(arg)
        elif opt in ("-j", "--jointIndex"):
            jointIndex = int(arg)            
        elif opt in ("-f", "--updateFreq"):
            updateFreq = float(arg)
        else:
            print "Unknown argument \"{0}\"".format(opt)

    print "Input parameters:\n"\
                  "  - joint pos task name: {0}\n"\
                  "  - final position: {1}\n"\
                  "  - desired velocity: {2}\n"\
                  "  - desired acceleration: {3}\n"\
                  "  - joint index: {4}\n"\
                  "  - update frequency: {5}".format(
        jointPosTaskName, pFinal, vDesired, aDesired, jointIndex, updateFreq)

    # Create the trajectory generator
    trajGen = SimpleSmoothJointTrajectoryGenerator(jointPosTaskName, pFinal, vDesired, aDesired, jointIndex, updateFreq)

    if not trajGen.init():
        rospy.signal_shutdown("done")
        sys.exit(-1)    

    index = raw_input("Start trajectory? Y/n\n")
    if index == "N" or index == "n":
        rospy.signal_shutdown("done")
        sys.exit(0)

    trajGen.start()

    print "Done publishing trajectory, waiting until ctrl+c is hit..."
    rospy.spin()  # just to prevent this node from exiting

    rospy.signal_shutdown("done")
