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

import rospy                        #for interacting with ROS topics and parameters
import sys, getopt                  #for parameters and sys.exit()
from Tkinter import *               #for GUI elements
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
import re                           #for findall()
import string                       #for split()
from controlit_core.srv import *    #for indices service (used to sort joint indices)

#initialize ROS node
rospy.init_node('GUIControl', anonymous=True)

width = input("Please input the number of columns desired: ")

# initialize Tkinter master
master = Tk()
master.title("Control GUI")

topics = rospy.get_published_topics(namespace = "/")
for topic in topics:
    robotname = re.findall("/(.+?)/JPosTask/+?",topic[0])
    if robotname !=[]:
        namespace = robotname[0]
        break

if (len(sys.argv) > 1): #publish to specified topic
    publishTopic = sys.argv[1]
else: #No argument, publish to joint goal position
    publishTopic = "/" + namespace + "/JPosTask/goalPosition" # set publisher location

print("publishing to " + publishTopic)

#initialize publisher
goalPublisher = rospy.Publisher(publishTopic, Float64MultiArray, queue_size = 0)


#The joint class will create the scale and entry elements in the GUI for each DOF
class Joint:
    nameList = []
    jointList = [] # contains all of the joint elements
    def __init__(self,Label,Min,Max,outFrame):
        self.frame = Frame(outFrame)
        self.scale = Scale(self.frame, from_=Min, to=Max, orient = HORIZONTAL, resolution = 0.01)
        self.scale.config(length = 600, label = Label)
        self.scale.bind()
        self.scale.pack(side = LEFT)
        self.en = Entry(self.frame)
        self.en.bind("<Return>", lambda e: self.setSlider()) #hitting ENTER in the entry box will run setSlider() on the joint
        self.en.config(width = 3)
        self.en.pack(side = LEFT)
        self.frame.pack(side = LEFT)
        Joint.nameList.append(Label)
        Joint.jointList.append(self)
        self.label = Label
        self.min = Min
        self.max = Max
    def getVal(self):
        return self.scale.get() # will return the scale value
    def setSlider(Joint):
        Joint.scale.set(float(Joint.en.get())) # get the entry value and set the slider to it
        Joint.en.delete(0,END) # clear the entry box

    def __str__(self):
        return "Joint: name = {0}, min = {1}, max = {2}".format(self.label, self.min, self.max)

    def __repr__(self):
        return self.__str__()

def getArray():
    tempArray=[]
    for el in sortedJointList:
        tempArray.append(el.getVal())
    return tempArray

urdf = rospy.get_param(namespace+"/controlit/robot_description")
nameList = re.findall("<joint name=\"(.+?)\">+?",urdf)
filteredNameList=[]
for el in nameList:
    if re.findall("type=\"fixed+?",el) == []:
        filteredNameList.append(el.split("\"",1)[0])
lowBoundList = re.findall(" lower=\"(.+?)\"+?",urdf)
highBoundList = re.findall(" upper=\"(.+?)\"+?",urdf)

index=0
while (index < len(filteredNameList)):
    fInd=0
    outFrame = Frame(master)
    while fInd < width and index < len(filteredNameList):
        Joint(filteredNameList[index],float(lowBoundList[index]),float(highBoundList[index]),outFrame)
        fInd+=1
        index+=1
    outFrame.pack()

getJointOrder = rospy.ServiceProxy(namespace +"/diagnostics/getRealJointIndices", get_parameters)
jointOrder = str(getJointOrder()) + "\n"
jointOrderList = re.findall(" value: (.+?)\n+?",jointOrder)
sortedJointList=[]
for el in jointOrderList:
    sortedJointList.append(Joint.jointList[Joint.nameList.index(el)])

initArray = getArray()

#create the MultiArray to be published
dim = MultiArrayDimension()
dim.size = len(initArray)
dim.label = "posMsg"
dim.stride = len(initArray)

goalArray = Float64MultiArray()
goalArray.data = initArray
goalArray.layout.dim.append(dim)
goalArray.layout.data_offset = 0

def updateGoal():
    if rospy.is_shutdown():
        sys.exit()
    goalArray.data = getArray()
    goalPublisher.publish(goalArray)
    master.after(50,updateGoal)

master.after(50,updateGoal) # update the goal every 50 ms (20 Hz)
mainloop()