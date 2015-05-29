#!/usr/bin/env python

import rospy		#for interacting with ROS
import sys, getopt	#for getting parameters and sys.exit()
import math			#for abs()
from std_msgs.msg import Float64MultiArray, MultiArrayDimension 
import time			#for sleep()
import threading	#for locks
import re			#for parsing

# define the method for generating a one dimensional MultiArray
def makeMultiArray(iterable, label):
	arrayList = []
	for el in iterable:
		arrayList.append(el)
	dim = MultiArrayDimension()
	dim.size = len(arrayList)
	dim.label = label
	dim.stride = len(arrayList)

	tempArray = Float64MultiArray()
	tempArray.data = arrayList
	tempArray.layout.dim.append(dim)
	tempArray.layout.data_offset = 0
	return tempArray

#initialize ROS node
rospy.init_node('listener', anonymous=True)

gLock=threading.Lock()	#goal position lock
pLock=threading.Lock()	#current position lock
vLock=threading.Lock()	#current velocity

dataLen = 0			#number of DOFs

#data lists
latestGoal = None	#input goal position
latestPos = None	#current position
latestVel = None	#current velocity
aAccelMsg = None	#command (output) acceleration
kpMsg = None		#proportional gain values (output for PD control)
kdMsg = None		#derivative gain values (output for PD control)

topics = rospy.get_published_topics(namespace = "/")
for topic in topics:
	robotname = re.findall("/(.+?)/JPosTask/+?",topic[0])
	if robotname !=[]:
		publishTopic = "/" + robotname[0] + "/JPosTask/" # set publisher/subscriber namespace
		break

if (len(sys.argv) < 6): #check to ensure the proper number of parameters has been provided
	print("Error! More parameters needed.\nPlease provide: topic kp kd a rate")
	sys.exit()
#assign the parameter values to the appropriate variables
kp = float(sys.argv[2])
kd = float(sys.argv[3])
a = float(sys.argv[4])
rate = float(sys.argv[5])

#Initialize the ROS publishers
goalPosPublisher = rospy.Publisher(publishTopic + "goalPosition", Float64MultiArray)
addAccelPublisher = rospy.Publisher(publishTopic + "goalAcceleration", Float64MultiArray)
kpPublisher = rospy.Publisher(publishTopic + "kp", Float64MultiArray, latch=True)
kdPublisher = rospy.Publisher(publishTopic + "kd", Float64MultiArray, latch=True)

def Goalcallback(data):
	gLock.acquire()
	if (latestGoal == None): #first time this callback is called
		global latestGoal
		latestGoal = makeMultiArray(data.data, "goalPos")
		global dataLen
		dataLen = len(data.data)
		global aAccelMsg
		aAccelMsg = makeMultiArray([0]*dataLen, "addAccel")
		global kpMsg
		kpMsg = makeMultiArray([0]*dataLen, "kp")
		global kdMsg
		kdMsg = makeMultiArray([0]*dataLen, "kd")
	else:
		latestGoal.data = data.data
	gLock.release()

def Actualcallback(data):
	pLock.acquire()
	if latestPos == None:
		global latestPos
		latestPos = makeMultiArray(data.data,"actualPos")
	else:
		latestPos.data = data.data
	pLock.release()

def Velocitycallback(data):
	vLock.acquire()
	if latestVel == None:
		global latestVel
		latestVel = makeMultiArray(data.data,"actualVel")
	else:
		latestVel.data = data.data
	vLock.release()

def setPath(r):
	goalRef = latestGoal.data
	kpMsg.data = [0]*dataLen
	kdMsg.data = [0]*dataLen
	kpPublisher.publish(kpMsg)
	kdPublisher.publish(kdMsg)
	goalPosPublisher.publish(latestGoal)
	while goalRef == latestGoal.data:
		aArray= []
		for ind in range(dataLen):
			dP = (latestGoal.data[ind] - latestPos.data[ind])
			vel = latestVel.data[ind]
			if abs(dP) > .3 or abs(vel) > .8:
				aDeterm = (vel * abs((vel)/(2*a))) 
				if aDeterm < dP:
					aArray.append(a)
				else:
					aArray.append(-a)
			else:
				kpMsg.data[ind] = kp
				kdMsg.data[ind] = kd
				aArray.append(0)
		aAccelMsg.data = aArray
		kpPublisher.publish(kpMsg)
		kdPublisher.publish(kdMsg)
		addAccelPublisher.publish(aAccelMsg)

		pLock.release()
		gLock.release()
		vLock.release()
		r.sleep()
		gLock.acquire()
		pLock.acquire()
		vLock.acquire()


if __name__ == '__main__':
	rospy.Subscriber(sys.argv[1], Float64MultiArray, Goalcallback)
	rospy.Subscriber(publishTopic + "actualPosition", Float64MultiArray, Actualcallback)
	rospy.Subscriber(publishTopic + "actualVelocity", Float64MultiArray, Velocitycallback)
	while latestGoal == None or latestPos == None or latestVel == None:
		print("Waiting for publishers")
		if latestVel == None or latestPos == None:
			print("Please make sure a Controlit! model is running.")
		if latestGoal == None:
			print("Please make sure a goal is being published.")
		rospy.sleep(0.1)
	print("no longer waiting")
	r = rospy.Rate(rate) # 20hz
	while not rospy.is_shutdown():
		gLock.acquire()
		pLock.acquire()
		vLock.acquire()
		setPath(r)
		pLock.release()
		gLock.release()
		vLock.release()
		r.sleep()