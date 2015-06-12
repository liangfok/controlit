/*
 * Copyright (C) 2015 The University of Texas at Austin and the
 * Institute of Human Machine Cognition. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 2.1 of
 * the License, or (at your option) any later version. See
 * <http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html>
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>
 */

#include <controlit/robot_interface_library/RobotInterfaceSM.hpp>

#include <chrono>
#include <controlit/Command.hpp>
#include <controlit/RTControlModel.hpp>
#include <controlit/logging/RealTimeLogging.hpp>
#include <controlit/robot_interface_library/OdometryStateReceiverSM.hpp>
#include <controlit/addons/ros/ROSMsgToString.hpp>

namespace controlit {
namespace robot_interface_library {

// Uncomment one of the following lines to enable/disable detailed debug statements.
#define PRINT_INFO_STATEMENT(ss)
// #define PRINT_INFO_STATEMENT(ss) CONTROLIT_INFO << ss;

#define PRINT_INFO_STATEMENT_RT(ss)
// #define PRINT_INFO_STATEMENT_RT(ss) CONTROLIT_INFO_RT << ss;

#define PRINT_INFO_RT(ss)
// #define PRINT_INFO_RT(ss) CONTROLIT_INFO_RT << ss;

#define PRINT_INFO_STATEMENT_RT_ALWAYS(ss) CONTROLIT_INFO_RT << ss;
// #define PRINT_INFO_STATEMENT_RT_ALWAYS(ss) std::cout << ss << std::endl;

#define PRINT_RECEIVED_STATE 0
#define PRINT_COMMAND 0

// Parameters for shared memory subscribers
#define LISTEN_TO_ROS_TOPIC false
#define USE_POLLING false

RobotInterfaceSM::RobotInterfaceSM() :
    RobotInterface(), // Call super-class' constructor
    receivedRobotState(false),
    robotStateSubscriber(LISTEN_TO_ROS_TOPIC, USE_POLLING),
    rttRxSubscriber(LISTEN_TO_ROS_TOPIC, USE_POLLING),
    rcvdJointState(false),
    isFirstReceptionState(true),
    isFirstReceptionRTT(true)
{
}

RobotInterfaceSM::~RobotInterfaceSM()
{
}

bool RobotInterfaceSM::init(ros::NodeHandle & nh, RTControlModel * model)
{
    PRINT_INFO_STATEMENT("Method called!");

    //---------------------------------------------------------------------------------
    // Initialize the parent class. Abort if parent class fails to initialize.
    //---------------------------------------------------------------------------------
    
    if(!RobotInterface::init(nh, model))
    {
        CONTROLIT_ERROR_RT << "Super-class failed to initialize.";
        return false;
    }

    //---------------------------------------------------------------------------------
    // Initialize a torque command message that will be used to hold the torque 
    // command data.
    //---------------------------------------------------------------------------------

    const std::vector<std::string> & names = model->get()->getRealJointNamesVector();

    PRINT_INFO_STATEMENT("Number of DoFs: " << names.size());

    torqueCmdMsg.layout.dim.resize(names.size());
    torqueCmdMsg.layout.dim[0].stride = names.size();
    torqueCmdMsg.layout.dim[0].size = names.size();
    torqueCmdMsg.layout.dim[1].stride = 1;
    torqueCmdMsg.layout.dim[1].size = 1;
    torqueCmdMsg.data.resize(names.size());

    //---------------------------------------------------------------------------------
    // Initialize the shared memory publshers and subscribers.  
    // There are two publishers:
    // 
    //   1. command publisher on topic "/cmd"
    //   2. seqno publisher on topic "/rtt_tx"
    //
    // There are two subscribers:
    //
    //   1. robot state subscriber on topic "/joint_states"
    //   2. seqno receiver on topic "/rtt_rx"
    //---------------------------------------------------------------------------------

    cmdPublisher.advertise("/cmd");
    
    rttTxPublisher.advertise("/rtt_tx");

    rttRxSubscriber.subscribe("/rtt_rx", boost::bind(&RobotInterfaceSM::rttCallback, this, _1));
    robotStateSubscriber.subscribe("/joint_states", boost::bind(&RobotInterfaceSM::jointStateCallback, this, _1));
    
    rttRxSubscriber.waitForMessage(rttRxMsg);
    robotStateSubscriber.waitForMessage(jointStateMsg);

    // int checkCount = 0;


    // while (!rttRxSubscriber.connected())
    // {
    //     // if (checkCount++ < 300)
    //     // {
    //     //     // ROS_WARN_THROTTLE(1.0, "Waiting for /rtt_rx shared memory topic.");
    //     //     if (checkCount % 10 == 0)
    //     //     {
    //     //     CONTROLIT_WARN << "Waiting for shared memory topic /rtt_rx to be connected....";
    //     //     }
    //     //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     // }
    //     // else
    //     // {
    //     //     CONTROLIT_ERROR << "Failed to subscribe to shared memory topic \"/rtt_rx\"!";
    //     //     return false;    
    //     // }   
    // }
    // CONTROLIT_INFO << "Subscription to /rtt_rx connected.";

    // checkCount = 0;
    // while (!robotStateSubscriber.connected())
    // {
    //     // if (checkCount++ < 300)
    //     // {
    //     //     // ROS_WARN_THROTTLE(1.0, "Waiting for /joint_states shared memory topic.");
    //     //     if (checkCount % 10 == 0)
    //     //     {
    //     //     CONTROLIT_WARN << "Waiting for shared memory topic /joint_states to be connected...";
    //     //     }
    //     //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     // }
    //     // else
    //     // {
    //     //     CONTROLIT_ERROR << "Failed to subscribe to shared memory topic \"/joint_states\"!";
    //     //     return false;    
    //     // }
    // }
    // CONTROLIT_INFO << "Subscription to /joint_states connected.";

    //---------------------------------------------------------------------------------
    // Initialize odometry receiver.
    //---------------------------------------------------------------------------------

    PRINT_INFO_STATEMENT("Creating and initializing the odometry state receiver...");
    odometryStateReceiver.reset(new OdometryStateReceiverSM());
    if (odometryStateReceiver->init(nh, model))
    {
        CONTROLIT_INFO << "Publishing dummy cmd and RTT TX messages...";
        cmdPublisher.publish(torqueCmdMsg);  // Send a dumy message to initialize shared memory connection
        rttTxPublisher.publish(currentRTTMsg);  // Send a dumy message to initialize shared memory connection
        return true;
    } else
        return false;


}

void RobotInterfaceSM::rttCallback(std_msgs::Int64 & msg)
{
    CONTROLIT_INFO_RT << "Method called!";

    if (isFirstReceptionRTT)
    {
        CONTROLIT_INFO_RT << "Received initial RTT message";
        isFirstReceptionRTT = false;
    }
    else
    {
        rttRxMsgMutex.lock();
        rttRxMsg = msg;
        rttRxMsgMutex.unlock();
    }
}

void RobotInterfaceSM::jointStateCallback(sensor_msgs::JointState & msg)
{
    CONTROLIT_INFO_RT << "Method called!";

    if (isFirstReceptionState)
    {
        CONTROLIT_INFO_RT << "Received initial robot state";
        isFirstReceptionState = false;
    }
    else
    {
        jointStateMutex.lock();

        jointStateMsg = msg;
        rcvdJointState = true;
    
        jointStateMutex.unlock();
    }
}

bool RobotInterfaceSM::read(controlit::RobotState & latestRobotState, bool block)
{
    bool result = true;

    //---------------------------------------------------------------------------------
    // Reset the timestamp within robot state to remember when the state was obtained.
    //---------------------------------------------------------------------------------
    latestRobotState.resetTimestamp();

    //---------------------------------------------------------------------------------
    // Check whether the sequence number was reflected.  If it was, compute and publish
    // the round trip communication latency.
    //---------------------------------------------------------------------------------
    rttRxMsgMutex.lock();
    bool seqnoReflected = (rttRxMsg.data == seqno);
    rttRxMsgMutex.unlock();

    if(seqnoReflected)
    {
        double latency = rttTimer->getTime();
        publishCommLatency(latency);
    }

    //---------------------------------------------------------------------------------
    // Process the latest joint state information.
    //---------------------------------------------------------------------------------

    jointStateMutex.lock();

    if (rcvdJointState)
    {
        //---------------------------------------------------------------------------------
        // Sanity check to make sure the received joint state information is of 
        // the correct length.
        //---------------------------------------------------------------------------------

        if(latestRobotState.getNumJoints() != jointStateMsg.name.size())
        {
            CONTROLIT_ERROR << "Received joint state message of incorrect size!\n"
                << "  - expected: " << latestRobotState.getNumJoints() << "\n"
                << "  - received: " << jointStateMsg.name.size();
            return false;
        }

        //---------------------------------------------------------------------------------
        // Intialize jointNameMap if it has not already been initalized.
        // This mapping stores each joint's index within the shared memory.
        //---------------------------------------------------------------------------------

        if (jointNameMap.size() == 0)
        {
            for (unsigned int ii = 0; ii < jointStateMsg.name.size(); ii++)
            {
                jointNameMap[jointStateMsg.name[ii]] = ii;
            }
        }

        //---------------------------------------------------------------------------------
        // Save the latest joint state information.
        //---------------------------------------------------------------------------------

        for(unsigned int ii = 0; ii < latestRobotState.getNumJoints(); ii++)
        {
            std::string name = latestRobotState.getJointNames()[ii];

            unsigned int shmIndx;
            try {
                shmIndx = jointNameMap.at(name);
            } 
            catch(std::out_of_range exception)
            {
                CONTROLIT_ERROR_RT << "Unknown joint name: " << name;
                result = false;
            }
            
            if (result)
            {
                // Update the joint state
                latestRobotState.setJointPosition(ii, jointStateMsg.position[shmIndx]);
                latestRobotState.setJointVelocity(ii, jointStateMsg.velocity[shmIndx]);
                latestRobotState.setJointEffort(ii, jointStateMsg.effort[shmIndx]);

                // TEMPORARY!
                if (std::abs(jointStateMsg.position[shmIndx]) > 1e3)
                {
                    std::cerr << "RobotInterfaceSM: questionable position: shmIndx = " << shmIndx << ", pos = " << jointStateMsg.position[shmIndx];
                    assert(false);
                }
            }
        }

        //---------------------------------------------------------------------------------
        // Check to ensure all receive states are valid. 
        // In this case, valid values are those less than 1000.
        // This is optional and can be commented out to increase execution speed.
        //---------------------------------------------------------------------------------

        {
            bool isValid = true;
            for(unsigned int ii = 0; ii < jointStateMsg.name.size() && isValid; ii++)            
            {
                if (std::abs(jointStateMsg.position[ii] >= 1e3)) isValid = false;
                if (std::abs(jointStateMsg.velocity[ii] >= 1e3)) isValid = false;
                if (std::abs(jointStateMsg.effort[ii] >= 1e3)) isValid = false;
            }

            if (!isValid)
                CONTROLIT_ERROR_RT 
                    << "Received questionable robot state:\n" 
                    << controlit::addons::ros::jointStateMsgToString(jointStateMsg);
        }

        
        #if PRINT_RECEIVED_STATE
        CONTROLIT_INFO_RT 
            << "Received state:\n"
            << controlit::addons::ros::jointStateMsgToString(jointStateMsg);;
        #endif
        
    } else
        result = false;

    jointStateMutex.unlock();

    //---------------------------------------------------------------------------------
    // Abort if we failed to receive joint state.
    //---------------------------------------------------------------------------------

    if (!result) return false;

    //---------------------------------------------------------------------------------
    // Get and save the latest odometry data.
    //---------------------------------------------------------------------------------
    if (!odometryStateReceiver->getOdometry(latestRobotState, block))
        return false;

    //---------------------------------------------------------------------------------
    // Call the the parent class' read method.  This causes the latestrobot state
    // to be published.
    //--------------------------------------------------------------------------------- 

    return controlit::RobotInterface::read(latestRobotState, block);
}

bool RobotInterfaceSM::write(const controlit::Command & command)
{
    //---------------------------------------------------------------------------------
    // Abort if jointNameMap is not initialized. It will be initialized the first
    // time a state message is received.
    //---------------------------------------------------------------------------------

    if (jointNameMap.size() == 0) return false;

    // The command may have fewer joints than the shared memory joint name map because
    // of unactuated joints.
    assert(torqueCmdMsg.data.size() >= command.getNumDOFs());
    
    // The command only applies to actuated joints
    const std::vector<std::string> & controlit_names = model->get()->getActuatedJointNamesVector();

    // Save the command into the message
    for(unsigned int ii = 0; ii < command.getNumDOFs(); ii++)
    {
        int shmIndx = jointNameMap[controlit_names[ii]]; //TODO: make sure the string is in the map!

        // CONTROLIT_INFO << "Saving command, shmIndx = " << shmIndx;
        // position_cmds[shmIndx] = command.getPositionCmd()[ii];
        // velocity_cmds[shmIndx] = command.getVelocityCmd()[ii];
        // torque_cmds[shmIndx] = command.getEffortCmd()[ii];
        torqueCmdMsg.data[shmIndx] = command.getEffortCmd()[ii];

        // #if PRINT_COMMAND
        // // ss << "  Joint: " << controlit_names[ii] << ", position_cmd: " << position_cmds[ii]
        // //    << ", velocity_cmd: " << velocity_cmds[ii] << ", torque_cmd: " << torque_cmds[ii] << std::endl;
        // #endif
    }

    // Print command (useful for debugging purposes)
    {
        #if PRINT_COMMAND
        std::stringstream ss;
        
        for(unsigned int ii = 0; ii < command.getNumDOFs(); ii++)
        {
            int shmIndx = jointNameMap[controlit_names[ii]];
            ss << "  Joint: " << controlit_names[ii] << ", effort_cmd: " << torqueCmdMsg.data[shmIndx] << std::endl;
        }

        std::cerr << "Writing command: " << std::endl << ss.str();
        #endif
    }

    cmdPublisher.publish(torqueCmdMsg);

    //---------------------------------------------------------------------------------
    // If necessary, send the next RTT sequence number.
    //---------------------------------------------------------------------------------

    if(sendSeqno)
    {
        sendSeqno = false;
        currentRTTMsg.data = ++seqno;
        rttTimer->start();
        rttTxPublisher.publish(currentRTTMsg);
    }

    //---------------------------------------------------------------------------------
    // Call the the parent class' write method.  This causes the command to be 
    // published.
    //--------------------------------------------------------------------------------- 

    return controlit::RobotInterface::write(command);
}

} // namespace robot_interface_library
} // namespace controlit
