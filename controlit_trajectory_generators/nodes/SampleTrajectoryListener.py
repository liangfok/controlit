#!/usr/bin/env python

""" 
An example trajectory listener that simply publishes the 
trajectories onto ROS topics.
"""

import rospy

from std_msgs.msg import Float64

class SampleTrajectoryListener:
    
    def __init__(self):
        """ 
        The constructor. It defines the messages and instantiates
        the ROS topic publishers for publishing the trajectory.
        """

        self.goalPosMsg = Float64()
        self.goalVelMsg = Float64()

        self.goalPosPublisher = rospy.Publisher("SampleTrajectoryListener/goalPosition", Float64, queue_size=1)
        self.goalVelPublisher = rospy.Publisher("SampleTrajectoryListener/goalVelocity", Float64, queue_size=1)

    def updateTrajGoals(self, goalPos, goalVel, done):

        self.goalPosMsg.data = goalPos
        self.goalPosPublisher.publish(self.goalPosMsg)

        self.goalVelMsg.data = goalVel
        self.goalVelPublisher.publish(self.goalVelMsg)