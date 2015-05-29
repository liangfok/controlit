#!/usr/bin/env python
# -*- coding: utf-8 -*-

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
Generates a square wave with a user-specified min/max values, number of DoFs, period, and ROS topic.
'''

import sys, getopt     # for getting and parsing command line arguments
import time
import math
import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

class SquareWaveGenerator:
    def __init__(self, rosTopic, minAmplitude, maxAmplitude, numDoFs, period):
        """
        The constructor.

        Keyword arguments:
        rosTopic -- The ROS topic on which to publish the goal.
        minAmplitude -- The minimum amplitude in radians.
        maxAmplitude  -- The maximum amplitude in radians.
        numDoFs -- The number of DoFs.
        period -- The period of the square wave in seconds.
        """

        self.period = period
        self.rosTopic = rosTopic

        # Define the dimensions of the message
        dim = MultiArrayDimension()
        dim.size = numDoFs
        dim.label = "goalMsg"
        dim.stride = 1

        # Define the goal message containing the upper bound of the square wave
        self.goalMsgHigh = Float64MultiArray()
        for ii in range(0, numDoFs):
            self.goalMsgHigh.data.append(maxAmplitude)
        self.goalMsgHigh.layout.dim.append(dim)
        self.goalMsgHigh.layout.data_offset = 0

        # Define the goal message containing the lower bound of the square wave
        self.goalMsgLow = Float64MultiArray()
        for ii in range(0, numDoFs):
            self.goalMsgLow.data.append(minAmplitude)
        self.goalMsgLow.layout.dim.append(dim)
        self.goalMsgLow.layout.data_offset = 0

    def start(self):
        """
        Starts the publishing of the square wave.
        """

        publisher = rospy.Publisher(self.rosTopic, Float64MultiArray)

        isHigh = False

        while not rospy.is_shutdown():

            if isHigh:
                publisher.publish(self.goalMsgLow)
                isHigh = False
            else:
                publisher.publish(self.goalMsgHigh)
                isHigh = True

            rospy.sleep(self.period)


# Main method
if __name__ == "__main__":

    rospy.init_node('SquareWaveGenerator', anonymous=True)

    # Define default values for the command line arguments
    rosTopic = '/JPosTask/goalPosition'
    minValue = -math.pi
    maxValue = math.pi
    numDoFs  = 3
    period   = 1.0

    usageStr = "Usage: python SquareWaveGenerator.py [parameters]\nValid parameters include:\n -t or --rosTopic [ros topic] (default {0})\n -l or --minAmplitude [min amplitude] (default {1})\n -u or --maxAmplitude [max amplitude] (default {2})\n -n or --numDoFs [number of DoFs] (default {3})\n -p or --Period [period] (default {4})".format(
        rosTopic, minValue, maxValue, numDoFs, period)

    # Parse the command line arguments
    try:
        opts, args = getopt.getopt(sys.argv[1:],"ht:l:u:n:p:",["rosTopic=", "minAmplitude=", "maxAmplitude=", "numDoFs=", "period="])
    except getopt.GetoptError:
       rospy.logerr(usageStr)
       sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            rospy.loginfo(usageStr)
            sys.exit()
        elif opt in ("-t", "--rosTopic"):
            rosTopic = arg
        elif opt in ("-l", "--minAmplitude"):
            minValue = float(arg)
        elif opt in ("-u", "--maxAmplitude"):
            maxValue = float(arg)
        elif opt in ("-n", "--numDoFs"):
            numDoFs = int(arg)
        elif opt in ("-p", "--period"):
            period = float(arg)
        else:
            print "Unknown argument \"{0}\"".format(opt)

    rospy.loginfo("Started with the following arguments:\n  - ROS topic: {0}\n  - Min Amplitude: {1}\n  - Max Amplitude: {2}\n  - Number of DoFs: {3}\n  - Period: {4}".format(
        rosTopic, minValue, maxValue, numDoFs, period))

    # Create a SquareWaveGenerator object
    squareWaveGenerator = SquareWaveGenerator(rosTopic, minValue, maxValue, numDoFs, period)
    squareWaveGenerator.start()