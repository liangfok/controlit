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
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension
from std_srvs.srv import Empty

class DemoChecking: 

  def __init__(self,robotPath, robotName, timeOut, stdOut, stdErr):
  

    self.robot_path = robotPath
    self.robot_name = robotName
  
    self.TIME_OUT = timeOut
    self.STDOUT = stdOut 
    self.STDERR = stdErr

    #For internal use
    self.FINAL_RESULT=[]
    self.CHECK_ERROR_RESULT = False
    self.joint_error_subscriber = None
    self.START_TIME = None
    self.LAUNCH = None
    
    #Set the new goal position
    self.dim = MultiArrayDimension(label ="x", size = 3, stride = 3)
    self.layout = MultiArrayLayout(dim = [self.dim] ,data_offset= 0)
    self.mulitarray = Float64MultiArray(layout = self.layout, data = [0.0, 0.6, 0.8])

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
    print os.getcwd()
    subprocess.call(["sh","generate_"+self.robot_name+"_urdfs.sh"],stdin=None, stdout=self.STDOUT, stderr=self.STDERR, shell=False)

    os.chdir(self.robot_path+"/hcrl_robot_controlit_configs/"+self.robot_name+"_controlit/models")
    print os.getcwd()
    subprocess.call(["sh","generate_"+self.robot_name+"_controlit_urdfs.sh"],stdin=None, stdout=self.STDOUT, stderr=self.STDERR, shell=False)

  def Kill_process(self):
    subprocess.call(["killall","rviz","gzclient","gzserver","rosout"],stdin=None, stdout=self.STDOUT, stderr=self.STDERR, shell=False)
    self.ROSCORE.terminate()
    self.LAUNCH.terminate()

  def Start_gazebo(self):
    rospy.wait_for_service("/stickbot_lowerleg_3dof_controller/gazebo/unpause_physics",timeout=self.TIME_OUT)
    self.start_gazebo = rospy.ServiceProxy("/stickbot_lowerleg_3dof_controller/gazebo/unpause_physics", Empty)
    self.start_gazebo()
    self.start_gazebo.close()
     

  def Wait_check_result(self):
    self.start_time = time.time()
    while ((time.time()-self.start_time) < self.TIME_OUT and self.CHECK_ERROR_RESULT == False):
      pass

  def Checkerror_callback(self,msg):
      if ((time.time()- self.START_TIME)>self.TIME_OUT):
        self.joint_error_subscriber.unregister() 
        print "NOT Ok" , str(self.check_number)+"/"+str(len(msg.data))
        print "Error: ",msg.data
        # rospy.signal_shutdown("Leave the spin");
        # pass
      self.check_number = 0
      # print "Joint Error: ",msg.data
      for ii in range(len(msg.data)):
        if (abs(msg.data[ii]) < 0.05):
          self.check_number +=1
      if (self.check_number == len(msg.data)):
        self.CHECK_ERROR_RESULT = True
        print "OK" , str(self.check_number)+"/"+str(len(msg.data))
        print "Error: ",msg.data
        self.joint_error_subscriber.unregister() 
        # rospy.signal_shutdown("Leave the spin");



  def Check_error(self):
    self.CHECK_ERROR_RESULT= False
    # For Subscriber
    self.START_TIME = time.time()
    self.joint_error_subscriber = rospy.Subscriber("/"+self.robot_name+"_controller/JPosTask/error", Float64MultiArray, self.Checkerror_callback)
    # rospy.spin() # MUST USE SPIN !! 
    # self.joint_error_subscriber.unregister()       
    # print self.CHECK_ERROR_RESULT
    # return self.CHECK_ERROR_RESULT

  def Pub_new_goal_position(self):
    self.start_pub_time = time.time()
    self.new_point_publisher = rospy.Publisher('/'+self.robot_name+'_controller/JPosTask/goalPosition', Float64MultiArray)

    while ((time.time() - self.start_pub_time) < 2):
      self.new_point_publisher.publish(self.mulitarray)
    self.new_point_publisher.unregister()




  def Combine_test(self):
    self.LAUNCH = subprocess.Popen(["roslaunch" ,self.robot_name+"_controlit" ,"simulate_jpos.launch"],stdin=None, stdout=self.STDOUT, stderr=self.STDERR, shell=False)
    time.sleep(self.TIME_OUT)
    self.Start_gazebo()

    print "Start doing Check1"
    self.Check_error() 
    self.Wait_check_result() # Wait the Golbal self.CHECK_ERROR_RESULT
    self.FINAL_RESULT.append (self.CHECK_ERROR_RESULT)

    print "Publish Another Goal Position"
    self.Pub_new_goal_position()

    print "Start doing Check2"
    self.Check_error()
    self.Wait_check_result() # Wait the Golbal self.CHECK_ERROR_RESULT
    self.FINAL_RESULT.append (self.CHECK_ERROR_RESULT)



    return self.FINAL_RESULT



# Main method
if __name__ == "__main__":

    # Define default values for the command line arguments
 
    robotPath = "/home/liang/controlit_hydro_workspace/"
    robotName = "stickbot_lowerleg_3dof"
    timeOut = 30
    stdOut = False 
    stdErr = False


    usageStr = "Usage: python test_stickbot_lowerleg_3dof.py [parameters]\n"\
               "Valid parameters include:\n"\
               " -p or --robotPath      (default {0})\n"\
               " -n or --robotName      (default {1})\n"\
               " -t or --timeOut        (default {2})\n"\
               " -o or --Display Output (default {3})\n"\
               " -e or --Display Error  (default {4})".format(robotPath, robotName, timeOut, stdOut, stdErr)

    # Parse the command line arguments
    try:
        opts, args = getopt.getopt(sys.argv[1:],"hp:n:t:oe",["robotPath=", "robotName=", "timeOut="])
    except getopt.GetoptError:
       print usageStr
       sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            print usageStr
            sys.exit()
        elif opt in ("-p", "--robotPath"):
            robotPath = arg
        elif opt in ("-n", "--robotName"):
            robotName = arg
        elif opt in ("-t", "--timeOut"):
            timeOut = float(arg)
        elif opt == "-o":
            stdOut = True
        elif opt == "-e":
            stdErr = True
        else:
            print "Unknown argument \"{0}\"".format(opt)

    print("Started with the following arguments:\n"\
                  "  - robotPath: {0}\n"\
                  "  - robotName: {1}\n"\
                  "  - timeOut:   {2}\n"\
                  "  - stdOut:    {3}\n"\
                  "  - stdErr:    {4}".format(robotPath, robotName, timeOut, stdOut, stdErr))


    #Create Object
    testobj = DemoChecking (robotPath, robotName, timeOut, stdOut, stdErr)
    #Generate urdf
    testobj.URDF_generator()
    # Test
    combine_result = testobj.Combine_test()
    print "Testing Result: ", combine_result
    testobj.Kill_process()
    