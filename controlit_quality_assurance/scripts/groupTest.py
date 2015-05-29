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

import sys, getopt     # for getting and parsing command line arguments
import os 
import subprocess
import time
import rospy
import rosnode
from controlit_core.srv import get_parameters
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension
from std_srvs.srv import Empty
from rosgraph_msgs.msg import Clock
# from diagnostic_msgs.msg import DiagnosticArray # To check the ready state of controller, but it is not work for all demo
from std_msgs.msg import Float64
import signal
import threading
import yaml

class DemoChecking: 

  def __init__(self,robotPath, robotName,robotNextGoalPosition, timeOut, errorThreshold, stdOut, stdErr):
  
    self.robot_path = robotPath
    self.robot_name = robotName
    self.robotNextGoalPosition = robotNextGoalPosition
  
    self.ERROR_THRESHOLD = errorThreshold

    self.TIME_OUT = timeOut
    self.STDOUT = stdOut 
    self.STDERR = stdErr

    self.LOCK = threading.Lock()   

    #For internal use
    self.CHECK_NUMBER = 0
    self.CHECK_ERROR_RESULT = False
    self.CHECK_ERROR_MESSAGE = {}
    self.FINAL_RESULT={}
    self.START_TIME = None
    self.READY_ERROR_MESSAGE = False
    self.CHANGE_NEW_POSITION = False

    self.CHECK_GAZEBO_STATE = False
    self.CHECK_CONTROLLER_STATE = False
    # self.LAUNCH = None
    
    #Initialization
    self.STDOUT = self.Set_shell_output(self.STDOUT)
    self.STDERR = self.Set_shell_output(self.STDERR)

    self.ROSCORE = subprocess.Popen(["roscore"],stdin=None, stdout=self.STDOUT, stderr=self.STDERR, shell=False)
    rospy.init_node('Hello_Testing', anonymous=True)


  def Set_shell_output(self,STD_):
    if (STD_ == True):
      STD_ = None
    else:
      STD_ = subprocess.PIPE
    return STD_

  def URDF_generator(self):
    print "Generating URDF"
    os.chdir(self.robot_path+"/hcrl_robot_models/"+self.robot_name+"/models")
    # print os.getcwd()
    subprocess.call(["sh","generate_"+self.robot_name+"_urdfs.sh"],stdin=None, stdout=self.STDOUT, stderr=self.STDERR, shell=False)

    os.chdir(self.robot_path+"/hcrl_robot_controlit_configs/"+self.robot_name+"_controlit/models")
    # print os.getcwd()
    subprocess.call(["sh","generate_"+self.robot_name+"_controlit_urdfs.sh"],stdin=None, stdout=self.STDOUT, stderr=self.STDERR, shell=False)

  def Kill_process(self):
    subprocess.call(["killall","controlit_exec","rviz","gzclient","gzserver","rosout","roscore","robot_state_publisher"],stdin=None, stdout=self.STDOUT, stderr=self.STDERR, shell=False)
    self.ROSCORE.terminate()

  def Start_gazebo(self):
    # Wait until Time out , Then error from ROS
    # rospy.wait_for_service("/"+self.robot_name+"_controller/gazebo/unpause_physics",timeout=self.TIME_OUT)

    self.start_gazebo = rospy.ServiceProxy("/"+self.robot_name+"_controller/gazebo/unpause_physics", Empty)
    try:
      self.start_gazebo()
    except rospy.ServiceException as errormsg:
      # print("Service did not process request: " + str(errormsg))
      pass
    self.start_gazebo.close()


  def Check_error_callback(self,msg):
    self.LOCK.acquire()
    self.READY_ERROR_MESSAGE = True
    self.CHECK_ERROR_MESSAGE["joint_error"] = msg.data
    self.LOCK.release()

  def Check_error(self,testName):
    self.CHECK_ERROR_RESULT= False
    self.START_TIME = time.time()
    # For Subscriber
    self.checking_status = False
    self.READY_ERROR_MESSAGE = False
    self.joint_error_subscriber = rospy.Subscriber("/"+self.robot_name+"_controller/JPosTask/error", Float64MultiArray, self.Check_error_callback)

    self.num_joint_pass_th = None
    while ( not(self.checking_status) and (time.time()- self.START_TIME)<self.TIME_OUT):
      self.LOCK.acquire()
      if (self.CHANGE_NEW_POSITION):
        # print "SEND NEW POSITION AGAIN"
        # self.Pub_new_goal_position()
        break
        # pass

      else:
        if (self.READY_ERROR_MESSAGE):
          self.CHECK_NUMBER = 0
          for ii in range(len(self.CHECK_ERROR_MESSAGE["joint_error"])):
            if (abs(self.CHECK_ERROR_MESSAGE["joint_error"][ii]) < self.ERROR_THRESHOLD):
              self.CHECK_NUMBER +=1

          self.num_joint_pass_th = str(self.CHECK_NUMBER)+"/"+str(len(self.CHECK_ERROR_MESSAGE["joint_error"]))
          self.CHECK_ERROR_MESSAGE["num_joint_pass_th"] = self.num_joint_pass_th

          if (self.CHECK_NUMBER == len(self.CHECK_ERROR_MESSAGE["joint_error"])):
            self.CHECK_ERROR_RESULT = True
            print "OK" , self.num_joint_pass_th
            self.checking_status = True
      self.LOCK.release()
      time.sleep(0.001)

    if (self.checking_status == False): # NOT ALL JOINT ERROR BELOW THRESHOLD
      if self.CHANGE_NEW_POSITION:
        print "NOT OK, Cannot change new position"
        self.CHECK_ERROR_MESSAGE["num_joint_pass_th"]=None
        self.CHECK_ERROR_MESSAGE["joint_error"]=None
        self.FINAL_RESULT["error_message"] = "Cannot change to new position)"
        self.READY_ERROR_MESSAGE = True
      else:
          print "NOT OK" , self.num_joint_pass_th
    
    self.joint_error_subscriber.unregister() 

    if (self.READY_ERROR_MESSAGE):
      self.CHECK_ERROR_MESSAGE["result"] = self.CHECK_ERROR_RESULT 
      self.FINAL_RESULT[testName] =dict(self.CHECK_ERROR_MESSAGE)
    else:
      self.FINAL_RESULT["status"] = False
      self.FINAL_RESULT["error_message"] = "Checking fail (Cannot read any /JPosTask/error message)"


  def Check_gazebo_state_callback(self,msg):
      self.CHECK_GAZEBO_STATE = True
      
  def Check_gazebo_state(self):
    self.CHECK_GAZEBO_STATE= False
    self.start_time = time.time()
    # For Subscriber
    self.clock_subscriber = rospy.Subscriber("/clock", Clock, self.Check_gazebo_state_callback)
    while ( not(self.CHECK_GAZEBO_STATE) and  ((time.time()-self.start_time) < self.TIME_OUT) ):
      time.sleep(0.001)
      self.Start_gazebo() 
      # pass
    self.clock_subscriber.unregister() 

    if (not (self.CHECK_GAZEBO_STATE)):
      print "ERROR: Gazebo is not ready"
      self.FINAL_RESULT["status"] = False
      self.FINAL_RESULT["error_message"] = "Gazebo is not ready"

    return self.CHECK_GAZEBO_STATE

  # USE ROSTOPIC TO CHECK - OLD_METHOD 
  def Check_controller_state_callback(self,msg):
      self.CHECK_CONTROLLER_STATE = True
      
  def Check_controller_state(self):
    self.CHECK_CONTROLLER_STATE = False
    self.start_time = time.time()
    # For Subscriber
    self.diagnostics_subscriber = rospy.Subscriber('/'+self.robot_name+'_controller/diagnostics/servoFrequency', Float64, self.Check_controller_state_callback)
    while ( not(self.CHECK_CONTROLLER_STATE) and  ((time.time()-self.start_time) < self.TIME_OUT) ):
      time.sleep(0.001)
    self.diagnostics_subscriber.unregister() 

    if (not(self.CHECK_CONTROLLER_STATE)):
      print "ERROR: Contoller is not ready"
      self.FINAL_RESULT["status"] = False     
      self.FINAL_RESULT["error_message"] = "Controller is not ready"
    return self.CHECK_CONTROLLER_STATE

  # # USE ROSNODE TO CHECK - NEW_METHOD
  # def Check_controller_state(self):
  #   self.CHECK_CONTROLLER_STATE = False
  #   self.start_time = time.time()
  #   # For Subscriber
  #   while ( not(self.CHECK_CONTROLLER_STATE) and  ((time.time()-self.start_time) < self.TIME_OUT) ):
  #     self.CHECK_CONTROLLER_STATE=rosnode.rosnode_ping(node_name="/"+self.robot_name+"_controller/controlit_exec",max_count=1, verbose=False) 
  #   return self.CHECK_CONTROLLER_STATE  


  def Pub_new_goal_position(self):
    self.start_pub_time = time.time()
    self.new_point_publisher = rospy.Publisher('/'+self.robot_name+'_controller/JPosTask/goalPosition', Float64MultiArray,queue_size = 1,latch=True)

    # while ((time.time() - self.start_pub_time) < 2): #Hard coding for keeping publishing the new_goal_position
    #   self.new_point_publisher.publish(self.robotNextGoalPosition)
    #   time.sleep(0.001)

    self.new_point_publisher.publish(self.robotNextGoalPosition)
    time.sleep(2)

    self.check_new_point_status_bool=True
    self.check_new_point_status_service = rospy.ServiceProxy('/'+self.robot_name+'_controller/diagnostics/getTaskParameters', get_parameters)
    try:
      self.get_check_message= self.check_new_point_status_service()
      # print self.get_check_message.params.values
      for iii in self.get_check_message.params.values:
        # print  iii.key
        if iii.key == "JPosTask/goalPosition":
          self.string_list=iii.value.replace('[',"").replace(']',"").split(",")
          for jjj in range(len(self.string_list)):
            if float(self.robotNextGoalPosition.data[jjj]) != float(self.string_list[jjj]):
              self.check_new_point_status_bool=False
              print "FAIL TO CHANGE NEW JOINT POSITION"
              break
          break
    except rospy.ServiceException as errormsg:
      print "FAIL TO GET DIAGNOSTIC MESSAGE"
    self.check_new_point_status_service.close()
    if self.check_new_point_status_bool:
      print "CAN CHANGE TO NEW POSITION"
      self.CHANGE_NEW_POSITION = False

    self.new_point_publisher.unregister()

  def Combine_test(self):
    self.FINAL_RESULT["robot_name"] = self.robot_name
    self.LAUNCH = subprocess.Popen(["roslaunch" ,self.robot_name+"_controlit" ,"simulate_jpos.launch"],stdin=None, stdout=self.STDOUT, stderr=self.STDERR, shell=False)
    print "Starting the controller"
    if (self.Check_controller_state()):
      print "Starting the Gazebo"    
      if (self.Check_gazebo_state()):
        self.FINAL_RESULT["status"] = True

        print "Checking 1 (Default Value)"
        self.Check_error("default_test") 

        if (self.robotNextGoalPosition != None):
          print "Publish Another Goal Position"
          self.CHANGE_NEW_POSITION = True
          self.Pub_new_goal_position()
          print "Checking 2 (New Goal Position)"
          self.Check_error("next_goal_test")
          time.sleep(5)

    # print "------------END----------- "
    # print self.FINAL_RESULT

    self.LAUNCH.send_signal(signal.SIGINT)
    print "Waiting for process to terminate"
    self.LAUNCH.communicate() # Wait for the process to terminate
    
    return self.FINAL_RESULT


# Main method
if __name__ == "__main__":

    # Define default values for the command line arguments
    ros_workspace = os.popen("echo $ROS_WORKSPACE")
    robotPath = ros_workspace.read().replace("\n","")
    timeOut = 20
    errorThreshold = 0.05
    stdOut = True 
    stdErr = True

    usageStr = "\nUsage: python test_stickbot_lowerleg_3dof.py [parameters]\n"\
               "Valid parameters include:\n"\
               " -p or --robotPath      (default {0})\n"\
               " -t or --timeOut        (default {1})\n"\
               " -j or --errorThreshold (default {2})\n"\
               " -o or --Display Output (default {3})\n"\
               " -e or --Display Error  (default {4})\n".format(robotPath, timeOut, errorThreshold, stdOut, stdErr)

    # Parse the command line arguments
    try:
        opts, args = getopt.getopt(sys.argv[1:],"hp:n:t:j:oe",["robotPath=", "timeOut=", "errorThreshold"])
    except getopt.GetoptError:
       print usageStr
       sys.exit()

    for opt, arg in opts:
        if opt == '-h':
            print usageStr
            sys.exit()
        elif opt in ("-p", "--robotPath"):
            robotPath = arg
        elif opt in ("-t", "--timeOut"):
            timeOut = float(arg)        
        elif opt in ("-j", "--errorThreshold"):
            errorThreshold = float(arg)
        elif opt == "-o":
            stdOut = False
        elif opt == "-e":
            stdErr = False
        else:
            print "Unknown argument \"{0}\"".format(opt)

    print
    print("Started with the following arguments:\n"\
                  "  - robotPath:       {0}\n"\
                  "  - timeOut:         {1}\n"\
                  "  - errorThreshold:  {2}\n"\
                  "  - stdOut :         {3}\n"\
                  "  - stdErr:          {4}".format(robotPath, timeOut, errorThreshold, stdOut, stdErr))
    print

    ############################################
    # originalRobotList = [
    # "atlas_legs_plain",
    # "atlas_plain",
    # "atlas_plain_pinned",
    # "dreamer",
    # "dreamer_no_left_arm",
    # "dreamer_testbed",
    # "stickbot_bipedal_12dof",
    # "stickbot_humanoid_32dof",
    # "stickbot_humanoid_32dof_pinned",
    # "stickbot_leg_6dof",
    # "stickbot_lowerleg_3dof",
    # "stickbot_upperbody_10dof",
    # "trikey",
    # "trikey_pinned",
    # ]

    # skipRobotList =[
    # # "atlas_plain",
    # # "dreamer",
    # # "dreamer_no_left_arm",
    # # "dreamer_testbed",

    # "atlas_plain_pinned",
    # "dreamer",
    # "dreamer_no_left_arm",
    # "dreamer_testbed",
    # "stickbot_bipedal_12dof",
    # "stickbot_humanoid_32dof",
    # "stickbot_humanoid_32dof_pinned",
    # "stickbot_leg_6dof",
    # "stickbot_lowerleg_3dof",
    # "stickbot_upperbody_10dof",
    # "trikey",
    # "trikey_pinned",
    # ]


    originalRobotList = []
    skipRobotList = []
    yaml_path = robotPath+"/controlit/controlit_quality_assurance/scripts/testing_list.yaml"
    with open(yaml_path, 'r') as testing_list_yaml:
      testing_dict = yaml.load(testing_list_yaml)
      for key in testing_dict:
        if (key == "available_demo"):
            originalRobotList = testing_dict[key]
            if originalRobotList == None or originalRobotList==[None]:
              print "Nothing in the available_demo! Bye."
              sys.exit()

        elif (key == "skip_test_list"):
            skipRobotList = testing_dict[key]
            if originalRobotList == None or originalRobotList==[None]:
              skipRobotList=[]
        else:
          print "Wrong yaml file!! Please correct it!"
          print "The key of the yaml file should contain 'available_demo' & 'skip_test_list' ONLY"
          print
          sys.exit()

    # Create the check list
    robotList = list(originalRobotList)

    for skipRobot in skipRobotList:
      try:
        robotList.remove(skipRobot)
      except:
        pass

    if robotList==[]:
      print "-- Nothing in the Checking List --"
    else:
      print "---- Start of Checking List ----"
      for robotName in robotList:
        print robotName 
      print
    if skipRobotList==[]:
      print "-- Nothing in the Skipping List --"
    else:
      print "----- Start of Skipping List -----"
      for skipRobot in skipRobotList:
        print skipRobot
    print
    ############################################  

    all_robot_test_result=[]
    for robotName in robotList: #USING LIST 
      print "----- Start Checking: ", robotName
      #Create Object # Test For Default value only

      # TESTING if need the next goal messagr for stickbot_lowerleg_3dof

      if robotName == "stickbot_lowerleg_3dof":
        dim = MultiArrayDimension(label ="x", size = 3, stride = 3)
        layout = MultiArrayLayout(dim = [dim] ,data_offset= 0)
        mulitarray = Float64MultiArray(layout = layout, data = [0.0, 0.6, 0.8])
        testobj = DemoChecking (robotPath=robotPath, robotName=robotName, robotNextGoalPosition=mulitarray, timeOut=timeOut, errorThreshold= errorThreshold, stdOut=stdOut, stdErr=stdErr)

      else:
        testobj = DemoChecking (robotPath=robotPath, robotName=robotName, robotNextGoalPosition=None, timeOut=timeOut, errorThreshold= errorThreshold, stdOut=stdOut, stdErr=stdErr)
      
      #Generate urdf
      testobj.URDF_generator()
      combine_result = testobj.Combine_test()
      print "Testing Result: ", combine_result

      all_robot_test_result.append(combine_result)
      testobj.Kill_process() 
      print "----  End " 
      print
      time.sleep(2)

    # print the result
    print "----------FULL REPORT-----------"
    OverallResult = True # Assume everything Ok, change to False if any error
    for robotResult in all_robot_test_result:

      print robotResult
      print

      print "robot_name: " , robotResult['robot_name']
      if (robotResult["status"] == False):
        print "    error_message", robotResult["error_message"]
        OverallResult = False
      else: # status = True
        print "    default_test: ", robotResult["default_test"]["result"], ", #Joint error below "+ str(errorThreshold),": ",robotResult["default_test"]["num_joint_pass_th"]  
        print "    Joint error: ", robotResult["default_test"]["joint_error"]
        if (robotResult["default_test"]["result"]==False):
          OverallResult = False
        
        # Need to Check if "next_goal_test" exists
        if ("next_goal_test" in robotResult):
          print "    next_goal_test: ", robotResult["next_goal_test"]["result"], ", #Joint error below "+ str(errorThreshold),": ",robotResult["next_goal_test"]["num_joint_pass_th"]
          print "    Joint error: ", robotResult["next_goal_test"]["joint_error"]
          if (robotResult["next_goal_test"]["result"]==False):
            OverallResult = False
      print

    print "-------END OF FULL REPORT-------"

    # For User Checking! Say OK if all demo works
    if (OverallResult == True):
      print "Every demo works perfect!"
    else:
      print "The following demo(s) fail(s):"
      for robotResult in all_robot_test_result:
        if (robotResult["status"] == False):
          print "    " , robotResult['robot_name'], ", error_message:", robotResult["error_message"]
        else:
          if (robotResult["default_test"]["result"]==False):
            print "    " , robotResult['robot_name'] 
            print "       -default_test" , ",#Joint error below "+ str(errorThreshold),": ",robotResult["default_test"]["num_joint_pass_th"] 
          # Need to Check if "next_goal_test" exists  
          if ("next_goal_test" in robotResult):
            if (robotResult["next_goal_test"]["result"]==False):
              print "       -next_goal_test ,#Joint error below "+ str(errorThreshold),": ",robotResult["next_goal_test"]["num_joint_pass_th"]
    print "-------------------------------"
    print "Final Tested Demo(s)"
    if robotList ==[]:
      print "    None"
    else:
      for robotName in robotList: #USING LIST 
        print "   ", robotName
    print "-------------------------------"
    print "Skipped Demo(s):"
    if skipRobotList==[]:
      print "    None"
    else:
      for skipRobot in skipRobotList:
        print "   ", skipRobot
    print "-------------------------------"




    # robotDict={}
    # #Set the new goal position FOR stickbot_lowerleg_3dof
    # name = "stickbot_lowerleg_3dof"
    # dim = MultiArrayDimension(label ="x", size = 3, stride = 3)
    # layout = MultiArrayLayout(dim = [dim] ,data_offset= 0)
    # mulitarray = Float64MultiArray(layout = layout, data = [0.0, 0.6, 0.8])
    # robotDict[name] = mulitarray

    # for robotKey,robotValue in robotDict.iteritems(): #USING DICTIONARY
    #   print "----- Start Checking: ", robotKey
    #   #Create Object
    #   testobj = DemoChecking (robotPath, robotKey, robotValue, timeOut, stdOut, stdErr)
    #   #Generate urdf
    #   testobj.URDF_generator()
    #   # Test
    #   combine_result = testobj.Combine_test()
    #   print "Testing Result: ", combine_result
    #   testobj.Kill_process()
    #   print "----  End " 
    #   print
    #   time.sleep(2)



# To do : (finish :1 ,2 )
# 1. Problem: Althougt the rosservice is ready, gazebo is not ready
#    Solution: Check the state of gazebo before sending the start signal. eg. Check rostopic
# 2. Problem: Gazebo start too fast, even before the controller is ready
#    Solution: Start the checking only if the controller is ready