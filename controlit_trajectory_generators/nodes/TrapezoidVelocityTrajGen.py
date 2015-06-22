#!/usr/bin/env python

'''
Generates a trajectory from the joint's current position to
its desired position. The velocity vs. time plot of the
trajectory looks like a trapezoid.

See: https://humancenteredrobotics.atlassian.net/wiki/display/WBC/2015.02.19+A+Trapezoid+Velocity+Trajectory+Generator
'''

import sys, getopt     # for getting and parsing command line arguments and exiting
import time
import math
import rospy           # for rospy.sleep()

def radToDeg(rad):
    return rad / 3.14 * 180

class TrapezoidVelocityTrajGen:
    def __init__(self):
        """
        The constructor.
        """

        self.isInitialized = False;

    def init(self, xInit, xFinal, vSteady, aAccel, aDecel, updateFreq):
        """
        Initializes this trajectory generator.

        Keyword arguments:
        xInit -- The initial position
        xFinal -- The final position
        vSteady -- The steady state velocity
        aAccel -- The desired acceleration
        aDecel -- The desired deceleration
        updateFreq -- The frequency at which the trajectory should be generated.

        Return: True if successful
        """

        # save parameters
        self.xInit = xInit
        self.xFinal = xFinal
        self.vSteady = vSteady
        self.aAccel = aAccel
        self.aDecel = aDecel
        self.updateFreq = updateFreq
        self.updatePeriod = 1.0 / updateFreq


        # Verify parameters are valid
        if self.aAccel == 0:
            print "TrapezoidVelocityTrajGen: init: ERROR: Invalid acceleration. Value must not be zero."
            return False

        if self.aDecel == 0:
            print "TrapezoidVelocityTrajGen: init: ERROR: Invalid acceleration. Value must not be zero."
            return False

        if self.vSteady == 0:
            print "TrapezoidVelocityTrajGen: init: ERROR: Invalid steady state velocity. Value must not be zero."
            return False

        # Ensure signs of variables are correct
        if self.xInit > self.xFinal:
            self.vSteady = -1 * abs(self.vSteady)
            self.aAccel = -1 * abs(self.aAccel)
            self.aDecel = abs(self.aDecel)
        else:
            self.vSteady = abs(self.vSteady)
            self.aAccel = abs(self.aAccel)
            self.aDecel = -1 * abs(self.aDecel)

        # Compute time spent acelerating and decelerating assuming vSteady is reached.
        tAccel = abs(self.vSteady / self.aAccel)
        tDecel = abs(self.vSteady / self.aDecel)

        # Compute the distance spent accelerating and decelerating assuming vSteady is reached.
        dAccel = (tAccel + tDecel) * abs(self.vSteady) / 2

        print "TrapezoidVelocityTrajGen: init: Input parameters:\n"\
              "  - steady state velocity = {0}\n"\
              "  - acceleration = {1}\n"\
              "  - deceleration = {2}\n"\
              "  - time accelerating = {3}\n"\
              "  - time decelerating = {4}\n"\
              "  - distance accelerating and decelerating = {5}\n"\
              "  - total distance = {6}".format(
                self.vSteady, self.aAccel, self.aDecel, tAccel, tDecel, dAccel,
                abs(self.xFinal - self.xInit))

        if dAccel > abs(self.xFinal - self.xInit):
            print "TrapezoidVelocityTrajGen: init: WARNING: Desired acceleration and deceleration prevents steady state velocity from being reached!\n"\
                  "  - steady state velocity: {0}\n"\
                  "  - acceleration time: {1}\n"\
                  "  - deceleration time: {2}\n"\
                  "  - acceleration distance: {3}\n"\
                  "  - deceleration distance: {4}\n"\
                  "  - total distance needed: {5}\n"\
                  "  - total distance to travel: {6}".format(
                    self.vSteady,
                    tAccel,
                    tDecel,
                    tAccel * abs(self.vSteady) / 2,
                    tDecel * abs(self.vSteady) / 2,
                    dAccel,
                    abs(self.xFinal - self.xInit))

            # Compute the constants
            self.tm = math.sqrt((2 * self.aDecel * (self.xInit - self.xFinal)) / (self.aAccel * (self.aAccel - self.aDecel)))

            self.c4 = (self.aAccel - self.aDecel) * self.tm

            self.t3 = self.c4 / (-1 * self.aDecel)

            self.c5 = self.xFinal - self.c4 * self.t3 - self.aDecel / 2 * math.pow(self.t3, 2)
            # print "Computation of c5:\n"\
            #       " - xFinal = {0}\n"\
            #       " - c4 = {1}\n"\
            #       " - t3 = {2}\n"\
            #       " - aDecel = {3}\n"\
            #       " - c5 = {4}".format(self.xFinal, self.c4, self.t3, self.aDecel, self.c5)

            # Convert case 2 constants to case 1 constants
            self.c1 = self.c4
            self.c3 = self.c5
            self.t1 = self.tm
            self.t2 = self.tm
            self.c2 = 0         # just a dummy value since c2 is not used when vSteady is not reached

        else:
            # Compute the steady state time
            tSteady = (abs(self.xFinal - self.xInit) - dAccel) / abs(self.vSteady)
            # print "Computation of tSteady:\n"\
            #       " - xFinal = {0}\n"\
            #       " - xInit = {1}\n"\
            #       " - dAccel = {2}\n"\
            #       " - vSteady = {3}\n"\
            #       " - tSteady = {4}".format(self.xFinal, self.xInit, dAccel, self.vSteady, tSteady)

            # Compute the time transition points
            self.t0 = 0
            self.t1 = tAccel
            self.t2 = self.t1 + tSteady
            self.t3 = self.t2 + tDecel

            # Compute the constants used in the position vs. time and velocity vs. time equations
            self.c1 = -1 * self.aDecel * self.t3
            self.c2 = self.aAccel / 2 * math.pow(self.t1, 2) + self.xInit - self.vSteady * self.t1
            self.c3 = self.vSteady * self.t2 + self.c2 - self.aDecel / 2 * math.pow(self.t2, 2) - self.c1 * self.t2

        print "TrapezoidVelocityTrajGen: init: computed time and constant variables:\n"\
               "  - t0 = 0\n"\
               "  - t1 = {0}\n"\
               "  - t2 = {1}\n"\
               "  - t3 = {2}\n"\
               "  - c1 = {3}\n"\
               "  - c2 = {4}\n"\
               "  - c3 = {5}".format(self.t1, self.t2, self.t3, self.c1, self.c2, self.c3)

        self.isInitialized = True;
        return True

    def start(self, listener):
        """
        Starts the publishing of the trajectory.

        Keyword arguments:
        listener -- The object receiving the trajectory.
        """

        if not self.isInitialized:
            print "TrapezoidVelocityTrajGen: start: ERROR: Unable to start trajectory generator before it is initialized."
            return False

        # Get the current time. The trajectory will be determined relative to this time point.
        t0 = rospy.get_rostime()

        print "TrapezoidVelocityTrajGen: start: Starting at time t0 = {0}\n".format(t0.to_sec())

        doneTraj = False
        while not rospy.is_shutdown() and not doneTraj:

            deltaTime = (rospy.get_rostime() - t0).to_sec()

            if deltaTime <= self.t1:
                goalPos = self.aAccel / 2 * math.pow(deltaTime, 2) + self.xInit
                goalVel = self.aAccel * deltaTime
            elif deltaTime <= self.t2 and not (self.t1 == self.t2):
                goalPos = self.vSteady * deltaTime + self.c2
                goalVel = self.vSteady
            elif deltaTime <= self.t3:
                goalPos = self.aDecel / 2 * math.pow(deltaTime, 2) + self.c1 * deltaTime + self.c3
                goalVel = self.aDecel * deltaTime + self.c1
            else:
                goalPos = self.xFinal
                goalVel = 0
                doneTraj = True

            listener.updateTrajGoals(goalPos, goalVel, doneTraj)

            if not doneTraj:
                rospy.sleep(self.updatePeriod)

        return True