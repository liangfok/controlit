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

#include <controlit/RobotInterface.hpp>

#include <controlit/Command.hpp>
#include <controlit/RobotState.hpp>
#include <controlit/RTControlModel.hpp>
#include <controlit/TimerROS.hpp>
#include <controlit/TimerChrono.hpp>

namespace controlit {

// Uncomment one of the following lines to enable/disable detailed debug statements.
#define PRINT_DEBUG_STATEMENT(ss)
// #define PRINT_DEBUG_STATEMENT(ss) CONTROLIT_DEBUG << ss;

#define PRINT_DEBUG_STATEMENT_RT(ss)
// #define PRINT_DEBUG_STATEMENT_RT(ss) CONTROLIT_DEBUG_RT << ss;

#define PRINT_INFO_STATEMENT_RT_ALWAYS(ss) CONTROLIT_INFO_RT << ss;
// #define PRINT_INFO_STATEMENT_RT_ALWAYS(ss) std::cout << ss << std::endl;

RobotInterface::RobotInterface() :
    rttLatencyPublisher("diagnostics/RTTCommLatency", 1),
    initialized(false),
    controllerName("DEFAULT_CONTROLLER_NAME"),
    model(nullptr),
    odometryStateReceiver(nullptr),
    seqno(0),
    sendSeqno(true),
    publishCounter(0),
    commandPublisher("diagnostics/command", 1),
    statePublisher("diagnostics/jointState", 1),
    useROSTimer(true)
{
}

RobotInterface::~RobotInterface()
{
    PRINT_DEBUG_STATEMENT("Method called.")
    PRINT_DEBUG_STATEMENT("Done method call.")
}

bool RobotInterface::init(ros::NodeHandle & nh, RTControlModel * model)
{
    PRINT_DEBUG_STATEMENT("Method called, initialized = " <<  (initialized ? "true" : "false"))

    //---------------------------------------------------------------------------------
    // Wait until robot_description becomes available on ROS parameter server.
    //---------------------------------------------------------------------------------

    while(!nh.hasParam("/robot_description") && ros::ok())
    {
        ROS_WARN_THROTTLE(2.0, "RobotInterface is waiting for robot description.");
    }

    //---------------------------------------------------------------------------------
    // Ensure this controller is only initialized once.
    //---------------------------------------------------------------------------------
    assert(!initialized);

    //---------------------------------------------------------------------------------
    // Save the controller's name and a pointer to the robot model.
    //---------------------------------------------------------------------------------

    controllerName = nh.getNamespace();
    this->model = model;

    //---------------------------------------------------------------------------------
    // Initialize the variables used to calculate the RTT communication latency.
    //---------------------------------------------------------------------------------

    seqno = 0;
    sendSeqno = true;
    rttTimer = getTimer();

    //---------------------------------------------------------------------------------
    // Initialize the publishers of the latest robot state and command.
    //---------------------------------------------------------------------------------
    if(statePublisher.trylock())
    {
        const std::vector<std::string> & jointNames = model->get()->getRealJointNamesVector();

        for (auto & name : jointNames)
        {
            statePublisher.msg_.name.push_back(name);
            statePublisher.msg_.position.push_back(0.0);  // allocate memory for the joint states
            statePublisher.msg_.velocity.push_back(0.0);
            statePublisher.msg_.effort.push_back(0.0);
        }

        statePublisher.unlockAndPublish();
    }
    else
    {
        CONTROLIT_ERROR << "Unable to initialize the state publisher!";
        return false;
    }

    if(commandPublisher.trylock())
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
        CONTROLIT_ERROR << "Unable to initialize the command publisher!";
        return false;
    }

    ros::param::param<bool>("controlit/use_ros_timer", useROSTimer, true);

    //---------------------------------------------------------------------------------
    // Mark the fact that this object is initialized.
    //---------------------------------------------------------------------------------

    initialized = true; // Record the fact that this RobotInterface was initialized

    PRINT_DEBUG_STATEMENT("Done method call.")

    return true;
}

bool RobotInterface::read(RobotState & latestRobotState, bool block)
{
    publishCounter++;

    if((publishCounter % 10 == 0) && statePublisher.trylock())
    {
        const Vector & jointPosition = latestRobotState.getJointPosition();
        const Vector & jointVelocity = latestRobotState.getJointVelocity();
        const Vector & jointEffort = latestRobotState.getJointEffort();

        for (size_t ii = 0; ii < latestRobotState.getNumJoints(); ii++)
        {
            statePublisher.msg_.position[ii] = jointPosition[ii];
            statePublisher.msg_.velocity[ii] = jointVelocity[ii];
            statePublisher.msg_.effort[ii] = jointEffort[ii];
        }
        statePublisher.unlockAndPublish();
    }
    return true;
}

bool RobotInterface::write(const Command & command)
{
    if ((publishCounter % 10 == 0) && commandPublisher.trylock())
    {
        
        const Vector & effortCmd = command.getEffortCmd();
        const Vector & posCmd    = command.getPositionCmd();
        const Vector & velCmd    = command.getVelocityCmd();

        for (size_t ii = 0; ii < command.getNumDOFs(); ii++)
        {

            commandPublisher.msg_.position[ii] = posCmd[ii];
            commandPublisher.msg_.velocity[ii] = velCmd[ii];
            commandPublisher.msg_.effort[ii]   = effortCmd[ii];
        }

        commandPublisher.unlockAndPublish();
    }
    return true;
}  


std::shared_ptr<Timer> RobotInterface::getTimer()
{
    std::shared_ptr<Timer> timerPtr;
    if (useROSTimer)
        timerPtr.reset(new TimerROS());
    else
        timerPtr.reset(new TimerChrono());
    return timerPtr;
}

void RobotInterface::publishCommLatency(double latency)
{
    // To be real-time safe, we only publish if we're able to obtain a lock on the publisher.
    if(rttLatencyPublisher.trylock())
    {
        rttLatencyPublisher.msg_.data = latency;
        rttLatencyPublisher.unlockAndPublish();
    }

    sendSeqno = true; // Should trigger the transmission of the next sequence number.
}

} // namespace controlit
