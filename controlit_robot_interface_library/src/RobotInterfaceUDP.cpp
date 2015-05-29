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


#include <controlit/robot_interface_library/RobotInterfaceUDP.hpp>

#include <chrono>
#include <controlit/RTControlModel.hpp>
#include <controlit/Command.hpp>
#include <controlit/addons/ros/ROSParameterAccessor.hpp>
#include <controlit/robot_interface_library/OdometryStateReceiverROSTopic.hpp>

namespace controlit {
namespace robot_interface_library {

// Uncomment one of the following lines to enable/disable detailed debug statements.
#define PRINT_DEBUG_STATEMENT(ss)
// #define PRINT_DEBUG_STATEMENT(ss) CONTROLIT_DEBUG << ss;

#define PRINT_DEBUG_STATEMENT_RT(ss)
// #define PRINT_DEBUG_STATEMENT_RT(ss) CONTROLIT_DEBUG_RT << ss;

#define PRINT_INFO_STATEMENT_RT_ALWAYS(ss) CONTROLIT_INFO_RT << ss;
// #define PRINT_INFO_STATEMENT_RT_ALWAYS(ss) std::cout << ss << std::endl;

RobotInterfaceUDP::RobotInterfaceUDP() :
    RobotInterface() // Call super-class' constructor
{
}

RobotInterfaceUDP::~RobotInterfaceUDP()
{
}

bool RobotInterfaceUDP::init(ros::NodeHandle & nh, RTControlModel * model)
{
    //---------------------------------------------------------------------------------
    // Initialize the parent class. Abort if parent class fails to initialize.
    //---------------------------------------------------------------------------------

    if (!RobotInterface::init(nh, model)) return false;

    //---------------------------------------------------------------------------------
    // Initialize the robot state and command messages.
    //---------------------------------------------------------------------------------

    rsMsg.state.num_dofs = model->get()->getRealJointNamesVector().size();
    cmdMsg.command.num_dofs = model->get()->getRealJointNamesVector().size();

    //---------------------------------------------------------------------------------
    // Get the UDP interface properties.
    //---------------------------------------------------------------------------------

    std::string address = DEFAULT_INET_ADDR;
    int cmdPort = DEFAULT_CMD_PORT;
    int statePort = DEFAULT_STATE_PORT;

    controlit::addons::ros::ROSParameterAccessor paramInterface(nh);

    if (!nh.getParam("RobotInterfaceUDP/address", address))
    {
        CONTROLIT_ERROR << "Unable to get address.";
        return false;
    }

    if (!nh.getParam("RobotInterfaceUDP/cmdPort", cmdPort))
    {
        CONTROLIT_ERROR << "Unable to get cmdPort.";
        return false;
    }

    if (!nh.getParam("RobotInterfaceUDP/statePort", statePort))
    {
        CONTROLIT_ERROR << "Unable to get statePort.";
        return false;
    }

    CONTROLIT_INFO << "Creating UDP interface with the following properties:\n"
             << "  - address: " << address << "\n"
             << "  - cmdPort: " << cmdPort << "\n"
             << "  - statePort: " << statePort;

    //---------------------------------------------------------------------------------             
    // Initialize the UDP interface.
    //---------------------------------------------------------------------------------

    if (!UDP.init(&cmdMsg, &rsMsg, (uint32_t)cmdPort, (uint32_t)statePort, address))
    {
        CONTROLIT_ERROR << "UDP initialization failed, returning false";
        return false;
    }

    //---------------------------------------------------------------------------------          
    // Initialize the odometry receiver.  
    // For now we receive this data from a ROS topic.
    //---------------------------------------------------------------------------------          

    CONTROLIT_INFO_RT << "Creating and initializing the odometry state receiver...";
    odometryStateReceiver.reset(new OdometryStateReceiverROSTopic());
    return odometryStateReceiver->init(nh, model);
}

bool RobotInterfaceUDP::read(controlit::RobotState & latestRobotState, bool block)
{
    //---------------------------------------------------------------------------------          
    // Sanity check to ensure the number of DOFs received is correct.
    //---------------------------------------------------------------------------------          

    if (rsMsg.state.num_dofs != latestRobotState.getNumJoints())
    {
        CONTROLIT_ERROR_RT << "Invalid number of DOFs in robot state message!\n"
            << " - rsMsg.state.num_dofs = " << rsMsg.state.num_dofs << "\n"
            << " - latestRobotState.getNumJoints() = " << latestRobotState.getNumJoints();
        return false;
    }


    //---------------------------------------------------------------------------------          
    // Check whether we have received a state message.  Block if necessary.
    //---------------------------------------------------------------------------------          

    if (rsMsg.seqno == -1)
    {
        if (!block)
        {
            //CONTROLIT_WARN_RT << "Did not receive robot state, aborting.";
            return false;
        }
        else
        {
            // Wait until message is received
            while (rsMsg.seqno == -1)
            {
                #define WAIT_DURATION_MS 100
                PRINT_DEBUG_STATEMENT("Waiting " << WAIT_DURATION_MS << " ms for robot state.")
                std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_DURATION_MS));
            }
        }
    }

    //---------------------------------------------------------------------------------
    // Reset the timestamp within robot state to remember when the state was obtained.
    //---------------------------------------------------------------------------------

    latestRobotState.resetTimestamp();

    //---------------------------------------------------------------------------------
    // Check whether the sequence number was reflected.  If it was, compute and publish
    // the round trip communication latency.
    //---------------------------------------------------------------------------------
    
    if (seqno == rsMsg.seqno)
    {
        double latency = rttTimer->getTime();
        publishCommLatency(latency);
    }

    //---------------------------------------------------------------------------------
    // Save the joint state information.
    // Assume the order of the joints in latestRobotState and rsMsg match.
    //---------------------------------------------------------------------------------

    for (unsigned int ii = 0; ii < latestRobotState.getNumJoints(); ii++)
    {
        latestRobotState.setJointPosition(ii, rsMsg.state.position[ii]);
        latestRobotState.setJointVelocity(ii, rsMsg.state.velocity[ii]);
        latestRobotState.setJointEffort(ii, rsMsg.state.effort[ii]);
    }
        
    //---------------------------------------------------------------------------------
    // Get and save the latest odometry data.
    //---------------------------------------------------------------------------------

    if (!odometryStateReceiver->getOdometry(latestRobotState, block))
        return false;

    PRINT_DEBUG_STATEMENT_RT("Read the following joint states:\n"
        " - q = " << latestRobotState.getJointPosition().transpose() << "\n"
        " - q_dot = " << latestRobotState.getJointVelocity().transpose() << "\n"
        " - effort = " << lastRobotState.getJointEffort().transpose());

    //---------------------------------------------------------------------------------
    // Call the the parent class' read method.  This causes the latestrobot state
    // to be published.
    //--------------------------------------------------------------------------------- 

    return controlit::RobotInterface::read(latestRobotState, block);
}

bool RobotInterfaceUDP::write(const controlit::Command & command)
{
    // HACK: The number of DOFs is hard-coded
    assert(cmdMsg.command.num_dofs == command.getNumDOFs());

    // Save the command into the message
    for (unsigned int ii = 0; ii < command.getNumDOFs(); ii++)
    {
        cmdMsg.command.effort[ii] = command.getEffortCmd()[ii];
    }

    if (sendSeqno)
    {
        sendSeqno = false;
        rttTimer->start();
        cmdMsg.seqno = ++seqno;
    }

    UDP.sendCommand();

    // Publish the command if the lock is available
    if(commandPublisher.trylock())
    {

        commandPublisher.msg_.header.stamp = ros::Time::now();
        for (unsigned int ii = 0; ii < command.getNumDOFs(); ii++)
        {
            commandPublisher.msg_.effort[ii] = command.getEffortCmd()[ii];
        }

        commandPublisher.unlockAndPublish();
    }

    //---------------------------------------------------------------------------------
    // Call the the parent class' write method.  This causes the command to be 
    // published.
    //--------------------------------------------------------------------------------- 

    return controlit::RobotInterface::write(command);
}


} // namespace robot_interface_library
} // namespace controlit
