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

#include <controlit/robot_interface_library/RobotInterfaceROSTopic.hpp>

#include <controlit/Command.hpp>
#include <controlit/RTControlModel.hpp>
#include <controlit_robot_interface_library/JointState.h>
#include <controlit/logging/RealTimeLogging.hpp>
#include <controlit/addons/ros/ROSMsgToStringRobotInterfaceLibrary.hpp>
#include <controlit/robot_interface_library/OdometryStateReceiverROSTopic.hpp>
// #include <chrono>

namespace controlit {
namespace robot_interface_library {

// Uncomment one of the following lines to enable/disable detailed debug statements.
#define PRINT_DEBUG_STATEMENT(ss)
// #define PRINT_DEBUG_STATEMENT(ss) CONTROLIT_DEBUG << ss;

#define PRINT_DEBUG_STATEMENT_RT(ss)
// #define PRINT_DEBUG_STATEMENT_RT(ss) CONTROLIT_DEBUG_RT << ss;

#define PRINT_INFO_STATEMENT_RT_ALWAYS(ss) CONTROLIT_INFO_RT << ss;
// #define PRINT_INFO_STATEMENT_RT_ALWAYS(ss) std::cout << ss << std::endl;

RobotInterfaceROSTopic::RobotInterfaceROSTopic() :
    RobotInterface(), // Call super-class' constructor
    receivedRobotState(false)
{
    activeRobotState = &robotState1;
    inactiveRobotState = &robotState2;
}

RobotInterfaceROSTopic::~RobotInterfaceROSTopic()
{
}

bool RobotInterfaceROSTopic::init(ros::NodeHandle & nh, RTControlModel * model)
{
    //---------------------------------------------------------------------------------
    // Initialize the parent class. Abort if parent class fails to initialize.
    //---------------------------------------------------------------------------------

    if (!RobotInterface::init(nh, model)) return false;

    //---------------------------------------------------------------------------------
    // Obtain the ROS topics for obtaining the robot state and transmitting the 
    // command.
    //---------------------------------------------------------------------------------
    
    // paramAccessor.reset(new controlit::addons::ros::ROSParameterAccessor(nh));

    std::string robotStateTopic;
    std::string robotCommandTopic;

    nh.param<std::string>("robot_state_topic", robotStateTopic, "robot_state");
    nh.param<std::string>("robot_command_topic", robotCommandTopic, "command");
    
    CONTROLIT_INFO << "Using topics: \n"
                   << "  - state: " << robotStateTopic << "\n"
                   << "  - command: " << robotCommandTopic;

    //---------------------------------------------------------------------------------
    // Create the ROS topic subscriber and publisher for joint states and commands.
    //---------------------------------------------------------------------------------

    // params: topic, queue size, callback method, object providing the callback method
    robotStateSubscriber = nh.subscribe(robotStateTopic, 1, & RobotInterfaceROSTopic::robotStateCallback, this);

    commandPublisher.init(nh, robotCommandTopic, 1);
    
    if (commandPublisher.trylock())
    {
        const std::vector<std::string> & jointNames = model->get()->getActuatedJointNamesVector();

        for (auto & name : jointNames)
        {
            commandPublisher.msg_.name.push_back(name);
            commandPublisher.msg_.position.push_back(0.0);  // allocate memory for the joint states
            commandPublisher.msg_.velocity.push_back(0.0);
            commandPublisher.msg_.effort.push_back(0.0);
        }

        commandPublisher.unlockAndPublish();
    }
    else
    {
        CONTROLIT_ERROR_RT << "Unable to initialize commandPublisher!";
        return false;
    }

    //---------------------------------------------------------------------------------
    // Initialize odometry receiver.
    //---------------------------------------------------------------------------------

    CONTROLIT_INFO_RT << "Creating and initializing the odometry state receiver...";
    odometryStateReceiver.reset(new OdometryStateReceiverROSTopic());
    return odometryStateReceiver->init(nh, model);
}

bool RobotInterfaceROSTopic::read(controlit::RobotState & latestRobotState, bool block)
{
    //---------------------------------------------------------------------------------          
    // Check whether we have received a state message.  Block if necessary.
    //---------------------------------------------------------------------------------   

    if (!receivedRobotState)
    {
        if (!block)
        {
            // supress warnings within first second of start time (only applies when simulated time is being used)
            if (ros::Time::now().toSec() > 1.0)
                CONTROLIT_WARN_RT << "Did not receive robot state, aborting.";
            return false;
        }
        else
        {
            // Wait until message is received
            while (!receivedRobotState)
            {
                #define WAIT_DURATION_MS 100
                PRINT_DEBUG_STATEMENT("Waiting " << WAIT_DURATION_MS << " ms for robot state message.")
                std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_DURATION_MS));
            }
        }
    }

    // We use try_lock() here to prevent the RT thread from blocking.
    if (mutex.try_lock())
    {
        //---------------------------------------------------------------------------------
        // Reset the timestamp within robot state to remember when the state was obtained.
        //---------------------------------------------------------------------------------

        latestRobotState.resetTimestamp();

        for (unsigned int ii = 0; ii < latestRobotState.getNumJoints(); ii++)
        {
            // Get the name of the current joint
            std::string name = latestRobotState.getJointNames()[ii];

            // Get the index of the joint in the robotState message
            int jointStateMsgIndex = -1;

            std::stringstream ss2;
            for (size_t jj = 0; jj < activeRobotState->name.size() && jointStateMsgIndex == -1; jj++)
            {
                ss2 << "Comparing " << activeRobotState->name[jj] << " with " << name << "...";
                if (activeRobotState->name[jj].compare(name) == 0)
                {
                    jointStateMsgIndex = (int)jj;
                    ss2 << "Match!" << std::endl;
                }
                else
                {
                    ss2 << "No match!" << std::endl;
                }
            }

            // Update the joint state
            if (jointStateMsgIndex != -1)
            {
                latestRobotState.setJointPosition(ii, activeRobotState->position[jointStateMsgIndex]);
                latestRobotState.setJointVelocity(ii, activeRobotState->velocity[jointStateMsgIndex]);
                latestRobotState.setJointEffort(ii, activeRobotState->effort[jointStateMsgIndex]);
            }
            else
            {
                std::stringstream ss;
                ss << "Unable to get index of joint \"" << name << "\" in JointStateMsg:" << std::endl;
                ss << controlit::addons::ros::jointStateMsgToString(*activeRobotState, "    ") << std::endl;
                ss << ss2.str() << std::endl;
                CONTROLIT_WARN_RT << ss.str();
            }
        }

        // Get and save the latest odometry data
        odometryStateReceiver->getOdometry(latestRobotState, block);

        mutex.unlock();
    }
    else
    {
        CONTROLIT_WARN_RT << "Unable to grab lock on mutex, not reading state this cycle";
    }


    //---------------------------------------------------------------------------------
    // Check to ensure joint states are valid. 
    // In this case, valid values are those less than 1000.
    // This is optional and can be commented out to increase execution speed.
    //---------------------------------------------------------------------------------

    if (!controlit::addons::eigen::checkMagnitude(latestRobotState.getJointPosition()) || !controlit::addons::eigen::checkMagnitude(latestRobotState.getJointVelocity()))
    {
        CONTROLIT_ERROR_RT << "Read in invalid joint states!.\n"
            << " - latestRobotState.getJointPosition(): " << latestRobotState.getJointPosition().transpose() << "\n"
            << " - latestRobotState.getJointVelocity(): " << latestRobotState.getJointVelocity().transpose();
        return false;
    }

    PRINT_DEBUG_STATEMENT_RT("Read the following joint states:\n"
                             " - q = " << latestRobotState.getJointPosition().transpose() << "\n"
                             " - q_dot = " << latestRobotState.getJointVelocity().transpose());


    //---------------------------------------------------------------------------------
    // Call the the parent class' read method.  This causes the latestrobot state
    // to be published.
    //--------------------------------------------------------------------------------- 

    return controlit::RobotInterface::read(latestRobotState, block);
}

bool RobotInterfaceROSTopic::write(const controlit::Command & command)
{
    if(commandPublisher.trylock())
    {
        // Verify dimensions match
        assert(commandPublisher.msg_.position.size() == (size_t)command.getPositionCmd().size());

        if (!controlit::addons::eigen::checkMagnitude(command.getPositionCmd())
            || !controlit::addons::eigen::checkMagnitude(command.getVelocityCmd())
            || !controlit::addons::eigen::checkMagnitude(command.getEffortCmd(), 1e3))
        {
            CONTROLIT_ERROR_RT << "Attempted to write invalid command!\n"
                << " - command.getPositionCmd(): " << command.getPositionCmd().transpose() << "\n"
                << " - command.getVelocityCmd(): " << command.getVelocityCmd().transpose() << "\n"
                << " - command.getEffortCmd(): " << command.getEffortCmd().transpose();
            assert(false);
            return false;
        }

        // Save the command into the message
        for (unsigned int ii = 0; ii < command.getNumDOFs(); ii++)
        {
            commandPublisher.msg_.position[ii] = command.getPositionCmd()[ii];
            commandPublisher.msg_.velocity[ii] = command.getVelocityCmd()[ii];
            commandPublisher.msg_.effort[ii] = command.getEffortCmd()[ii];
        }

        // Set the RTT sequence number
        if (sendSeqno)
        {
            sendSeqno = false;
            rttTimer->start();
            commandPublisher.msg_.rtt_seq_no = ++seqno;
        }

        // Publish the message
        commandPublisher.unlockAndPublish();
    }

    //---------------------------------------------------------------------------------
    // Call the the parent class' write method.  This causes the command to be 
    // published.
    //--------------------------------------------------------------------------------- 

    return controlit::RobotInterface::write(command);
}

void RobotInterfaceROSTopic::robotStateCallback(const boost::shared_ptr<controlit_robot_interface_library::JointState const> & robotState)
{

    //---------------------------------------------------------------------------------
    // Check whether the sequence number was reflected.  If it was, compute and publish
    // the round trip communication latency.
    //---------------------------------------------------------------------------------
    
    if (robotState->rtt_seq_no == seqno)
    {
        double latency = rttTimer->getTime();
        publishCommLatency(latency);
    }

    // Save the robot state in the local inactiveRobotState variable
    *inactiveRobotState = *(robotState.get());

    // grab lock on mutex, swap the active and inactive robot states, release the lock
    mutex.lock();

    std::swap(activeRobotState, inactiveRobotState);

    // release lock on mutex
    mutex.unlock();

    // Set a flag indicating that a robot state was received
    receivedRobotState = true;
}

} // namespace robot_interface_library
} // namespace controlit
