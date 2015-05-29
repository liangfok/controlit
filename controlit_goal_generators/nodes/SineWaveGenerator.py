#!/usr/bin/env python

###
 # Copyright (C) 2015 The University of Texas at Austin and the
 # Institute of Human Machine Cognition. All rights reserved.
 #
 # This program is free software: you can redistribute it and/or
 # modify it under the terms of the GNU Lesser General Public License
 # as published by the Free Software Foundation, either version 2.1 of
 # the License, or (at your option) any later version. See
 # <http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html>
 #
 # This program is distributed in the hope that it will be useful, but
 # WITHOUT ANY WARRANTY; without even the implied warranty of
 # MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 # Lesser General Public License for more details.
 #
 # You should have received a copy of the GNU Lesser General Public
 # License along with this program.  If not, see
 # <http://www.gnu.org/licenses/>
###

'''
Generates a sine wave with a user-specified min/max values, number of DoFs, period, and ROS topic.
'''

import sys, getopt     # for getting and parsing command line arguments
import time
import math
import threading
import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

class SineWaveGenerator:
    def __init__(self, rosTopic, amplitude, offset, initGoal, numDoFs, jointIndex, period, updateFreq):
        """
        The constructor.

        Keyword arguments:
        rosTopic -- The ROS topic on which to publish the goal.
        initGoal -- The initial goal (the value the sine wave starts at)
        amplitude -- The amplitude.
        offset  -- The offset.
        numDoFs -- The number of DoFs.
        jointIndex -- The index of the joint being controlled.
        period -- The period of the sine wave in seconds.
        updateFreq -- The frequency at which the reference position should be sent
        """

        self.period = period
        self.frequency = 1 / period
        self.rosTopic = rosTopic
        self.amplitude = amplitude
        self.offset = offset
        self.jointIndex = jointIndex
        self.updateFreq = updateFreq
        self.updatePeriod = 1.0 / updateFreq

        # Define the dimensions of the message
        dim = MultiArrayDimension()
        dim.size = numDoFs
        dim.label = "goalMsg"
        dim.stride = 1

        # Define the goal message
        self.goalMsg = Float64MultiArray()
        for ii in range(0, numDoFs):
            self.goalMsg.data.append(initGoal)
        self.goalMsg.layout.dim.append(dim)
        self.goalMsg.layout.data_offset = 0

    def setJointIndex(self, jointIndex):
        """
        Changes the joint that is being controlled.

        Keyword arguments:
        jointIndex -- The index of the joint to be controlled.
        """

        self.jointIndex = jointIndex

    def getSineSignal(self, elapsedTime_sec, amplitude, offset, freq_hz):
        return amplitude * math.sin(elapsedTime_sec * 2 * math.pi * freq_hz) + offset

    def start(self):
        """
        Starts the publishing of the sine wave.
        """

        publisher = rospy.Publisher(self.rosTopic, Float64MultiArray)
        startTime = time.time()

        while not rospy.is_shutdown():
            goal = self.getSineSignal(time.time() - startTime, self.amplitude, self.offset, self.frequency)

            # Make the joint at jointIndex move in a sine wave.
            # Set all other joints to be at position zero.
            for ii in range(0, numDoFs):
                self.goalMsg.data[ii] = 0
            self.goalMsg.data[self.jointIndex] = goal

            publisher.publish(self.goalMsg)
            rospy.sleep(self.updatePeriod)


# Main method
if __name__ == "__main__":

    rospy.init_node('SineWaveGenerator', anonymous=True)

    # Define default values for the command line arguments
    rosTopic = '/JPosTask/goalPosition'
    amplitude = 1
    offset = 0
    initGoal = 0
    numDoFs  = 1
    jointIndex = 0
    period   = 1.0
    updateFreq = 50

    usageStr = "Usage: python SineWaveGenerator.py [parameters]\n"\
               "Valid parameters include:\n"\
               " -t or --rosTopic [ros topic] (default {0})\n"\
               " -a or --amplitude [amplitude] (default {1})\n"\
               " -o or --offset [offset, the vertical offset of the sine waves] (default {2})\n"\
               " -i or --initGoal [init goal, the initial value of the sine waves] (default {3})\n"\
               " -n or --numDoFs [number of DoFs] (default {4})\n"\
               " -j or --jointIndex [joint index], the index of the joint to move in a sine wave (default {5}\n"\
               " -p or --period [period, the period of the sine wave] (default {6})\n"\
               " -f or --updateFreq [update frequency, the rate at which new points along the trajectory are published in Hz] (default {7})".format(
        rosTopic, amplitude, offset, initGoal, numDoFs, jointIndex, period, updateFreq)

    # Parse the command line arguments
    try:
        opts, args = getopt.getopt(sys.argv[1:],"ht:a:o:i:n:j:p:f:",["rosTopic=", "amplitude=", "offset=", "initGoal=", "numDoFs=", "jointIndex=", "period=", "updateFreq="])
    except getopt.GetoptError:
       rospy.logerr(usageStr)
       sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            rospy.loginfo(usageStr)
            sys.exit()
        elif opt in ("-t", "--rosTopic"):
            rosTopic = arg
        elif opt in ("-a", "--amplitude"):
            amplitude = float(arg)
        elif opt in ("-o", "--offset"):
            offset = float(arg)
        elif opt in ("-i", "--initGoal"):
            initGoal = float(arg)
        elif opt in ("-n", "--numDoFs"):
            numDoFs = int(arg)
        elif opt in ("-j", "--jointIndex"):
            jointIndex = int(arg)            
        elif opt in ("-p", "--period"):
            period = float(arg)
        elif opt in ("-f", "--updateFreq"):
            updateFreq = float(arg)
        else:
            print "Unknown argument \"{0}\"".format(opt)

    rospy.loginfo("Input parameters:\n"\
                  "  - ROS topic: {0}\n"\
                  "  - Amplitude: {1}\n"\
                  "  - Offset: {2}\n"\
                  "  - Init Goal: {3}\n"\
                  "  - Number of DoFs: {4}\n"\
                  "  - Joint Index: {5}\n"\
                  "  - Period: {6}\n"\
                  "  - Update Frequency: {7}".format(
        rosTopic, amplitude, offset, initGoal, numDoFs, jointIndex, period, updateFreq))

    # Create a SineWaveGenerator object
    sineWaveGenerator = SineWaveGenerator(rosTopic, amplitude, offset, initGoal, numDoFs, jointIndex, period, updateFreq)
    t = threading.Thread(target=sineWaveGenerator.start)
    t.start()

    done = False
    while not done:
        index = raw_input("Change sine wave to index (type q to exit): ")

        if "q" == index:
            done = True
        elif int(index) >= numDoFs:
            print "ERROR: Tried to set index to a value greater than the number of DOFs ({0})!".format(numDoFs)
        else:
            print "Setting joint index to be {0}".format(index)
            sineWaveGenerator.setJointIndex(int(index))

    rospy.signal_shutdown("done")
