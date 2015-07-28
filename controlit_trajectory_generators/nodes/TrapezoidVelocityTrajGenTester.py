#!/usr/bin/env python

"""
Tests a trapezoid velocity generator and publishes the
generated trajectory onto a ROS topic.
"""

import sys, getopt     # for getting and parsing command line arguments and exiting

import rospy

import TrapezoidVelocityTrajGen
import SampleTrajectoryListener

# Main method
if __name__ == "__main__":

    rospy.init_node('TrapezoidVelocityTrajectoryGeneratorTester', anonymous=True)

    # Define default values for the command line arguments
    xInit = 0
    xFinal = 1
    vSteady = 0.1
    aAccel = 0.01
    aDecel = -0.01
    updateFreq = 100

    usageStr = "TrapezoidVelocityTrajGenTester: Usage: python TrapezoidVelocityTrajGenTester.py [parameters]\n"\
               "Valid parameters include:\n"\
               " -i or --xInit [initial position] (default {0})\n"\
               " -f or --xFinal [final position] (default {1})\n"\
               " -v or --vSteady [the desired steady-state velocity] (default {2})\n"\
               " -a or --aAccel [the desired acceleration] (default {3})\n"\
               " -d or --aDecel [the desired deceleration] (default {4})\n"\
               " -f or --updateFreq [the rate at which new points along the trajectory are published in Hz] (default {5})".format(
        xInit, xFinal, vSteady, aAccel, aDecel, updateFreq)

    # Parse the command line arguments
    try:
        opts, args = getopt.getopt(sys.argv[1:],"hi:f:v:a:d:f:",["xInit=", "xFinal=",
            "vSteady=", "aAccel=", "aDecel=", "updateFreq="])
    except getopt.GetoptError:
       rospy.logerr(usageStr)
       sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            print usageStr
            sys.exit()
        elif opt in ("-i", "--xInit"):
            xInit = float(arg)
        elif opt in ("-f", "--xFinal"):
            xFinal = float(arg)
        elif opt in ("-v", "--vSteady"):
            vSteady = float(arg)
        elif opt in ("-a", "--aAccel"):
            aAccel = float(arg)
        elif opt in ("-d", "--aDecel"):
            aDecel = float(arg)
        elif opt in ("-f", "--updateFreq"):
            updateFreq = float(arg)
        else:
            print "TrapezoidVelocityTrajGenTester: ERROR: Unknown argument \"{0}\"".format(opt)

    print "TrapezoidVelocityTrajGenTester: Input parameters:\n"\
          "  - initial position: {0}\n"\
          "  - final position: {1}\n"\
          "  - steady state velocity: {2}\n"\
          "  - acceleration: {3}\n"\
          "  - deceleration: {4}\n"\
          "  - update frequency: {5}".format(
        xInit, xFinal, vSteady, aAccel, aDecel, updateFreq)

    # Create the trajectory generator
    trajGen = TrapezoidVelocityTrajGen.TrapezoidVelocityTrajGen()

    # Initialize the trajectory generator
    if not trajGen.init(xInit, xFinal, vSteady, aAccel, aDecel, updateFreq):
        rospy.signal_shutdown("done")
        sys.exit(-1)

    # Create the trajectory listener
    trajListener = SampleTrajectoryListener.SampleTrajectoryListener()

    index = raw_input("TrapezoidVelocityTrajGenTester: Start trajectory? Y/n\n")
    if index == "N" or index == "n":
        rospy.signal_shutdown("done")
        sys.exit(0)

    trajGen.start(trajListener)

    print "TrapezoidVelocityTrajGenTester: Done publishing trajectory, waiting until ctrl+c is hit..."
    rospy.spin()  # just to prevent this node from exiting

    rospy.signal_shutdown("done")
