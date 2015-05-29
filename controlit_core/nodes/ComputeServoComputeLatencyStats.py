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

import sys, getopt     # for getting and parsing command line arguments
import rospy
import numpy as np
import matplotlib.pyplot as plt

latencyRead = []
latencyPublishOdom = []
latencyModelUpdate = []
latencyComputeCommand = []
latencyEmitEvents = []
latencyWrite = []
latencyServo = []

def printLatencyStats(nameOfLatency, cache):
        print "{0} Latency Stats (ms): {1} ± {2}".format(nameOfLatency, np.average(cache) * 1000, np.std(cache) * 1000)
        print "  Number of Samples: {0}".format(len(cache))
        print "  Average: {0}".format(np.average(cache) * 1000)
        print "  Variance: {0}".format(np.var(cache) * 1000)
        print "  Standard Deviation: {0}".format(np.std(cache) * 1000)
        print "  Max: {0}".format(np.nanmax(cache) * 1000)
        print "  Min: {0}".format(np.nanmin(cache) * 1000)

if __name__ == '__main__':
    # Read in the data

    # Define default values for the command line arguments
    logFile = None
    trialNumber = None
    numSamples = 1000

    usageStr = "Usage: python {0} [parameters]\n"\
               "Valid parameters include:\n"\
               " -l or --logFile [log file]\n"\
               " -t or --trialNumber [trial number]\n"\
               " -n or --numSamples [number of samples to consider] (default {1})".format(
                __file__, numSamples)

    # Parse the command line arguments
    try:
        opts, args = getopt.getopt(sys.argv[1:],"hl:t:n:",["logFile=", "trialNumber=", "numSamples="])
    except getopt.GetoptError:
       rospy.logerr(usageStr)
       sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            print usageStr
            sys.exit()
        elif opt in ("-l", "--logFile"):
            logFile = arg
        elif opt in ("-t", "--trialNumber"):
            trialNumber = int(arg)
        elif opt in ("-n", "--numSamples"):
            numSamples = int(arg)
        else:
            rospy.logerr("Unknown argument \"{0}\"".format(opt))
            sys.exit(0)

    print "Input parameters:\n"\
          "  - log file: {0}\n"\
          "  - trial number: {1}\n"\
          "  - num samples: {2}\n".format(
        logFile, trialNumber, numSamples)

    if logFile == None or trialNumber == None:
        print "log file or trial number not specified!"
        sys.exit(0)

    ff = open(logFile, 'r')

    doneReading = False
    line = ff.readline()
    while line != "" and not doneReading:

        if "[" in line:
            line = line.lstrip('[')
            line = line.rstrip(']\n')
            latencyValues = line.split(',')
            # print latencyValues

            latencyRead.append(float(latencyValues[0]))
            latencyPublishOdom.append(float(latencyValues[1]))
            latencyModelUpdate.append(float(latencyValues[2]))
            latencyComputeCommand.append(float(latencyValues[3]))
            latencyEmitEvents.append(float(latencyValues[4]))
            latencyWrite.append(float(latencyValues[5]))
            latencyServo.append(float(latencyValues[6]))

        doneReading = (len(latencyRead) == numSamples)
        line = ff.readline()

    if len(latencyRead) != numSamples:
        print "ERROR: Insufficient number of data points. Got {0} need {1}".format(len(latencyRead), numSamples)
    else:
        printLatencyStats("Read", latencyRead)
        printLatencyStats("Publish Odometry", latencyPublishOdom)
        printLatencyStats("Update Model", latencyModelUpdate)
        printLatencyStats("Compute Command", latencyComputeCommand)
        printLatencyStats("Emit Events", latencyEmitEvents)
        printLatencyStats("Write", latencyWrite)
        printLatencyStats("Total servo compute time", latencyServo)

        # x = [i for i in xrange(len(latencyCache))]

        # fig, ax = plt.subplots()

        # latencyLine = ax.plot(x, latencyCache, 'ro')
        # # childThreadLine = ax.plot(childThreadTime, childThread, 'bx')

        # # add some labels
        # ax.set_title("Latency {0} (Trial {1})".format(sys.argv[1], trialNumber))
        # ax.set_ylabel('Latency (ms)')
        # ax.set_xlabel("Sample Number")

        # # ax.legend( (latencyLine[0]), ('main thread') )

        # # plt.ylim([-1,4])

        # plt.show()