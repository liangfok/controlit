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

#include <controlit/robot_interface_library/RobotInterfaceBenchmark.hpp>

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

RobotInterfaceBenchmark::RobotInterfaceBenchmark() :
    RobotInterface() // Call super-class' constructor
{
}

RobotInterfaceBenchmark::~RobotInterfaceBenchmark()
{
}

bool RobotInterfaceBenchmark::init(ros::NodeHandle & nh, RTControlModel * model)
{
    // Initialize the parent class.
    bool initSuccess = RobotInterface::init(nh, model);

    // If the super-class fails to initialize, abort.
    if (!initSuccess) return false;

    return true;
}

bool RobotInterfaceBenchmark::read(controlit::RobotState & latestRobotState, bool block)
{

    // Reset the timestamp within robot state to remember when the state was obtained.
    latestRobotState.resetTimestamp();

    // for (unsigned int ii = 0; ii < latestRobotState.getNumJoints(); ii++)
    // {
    //     latestRobotState.setJointPosition(ii, 0);
    //     latestRobotState.setJointVelocity(ii, 0);
    //     latestRobotState.setJointEffort(ii, 0);
    // }

    // PRINT_DEBUG_STATEMENT_RT("Read the following joint states:\n"
    //     " - q = " << latestRobotState.getJointPosition().transpose() << "\n"
    //     " - q_dot = " << latestRobotState.getJointVelocity().transpose());

    // if (!controlit::addons::eigen::checkMagnitude(latestRobotState.getJointPosition()) || !controlit::addons::eigen::checkMagnitude(latestRobotState.getJointVelocity()))
    // {
    //     CONTROLIT_ERROR_RT << "Read in invalid joint states!.\n"
    //         << " - latestRobotState.getJointPosition(): " << latestRobotState.getJointPosition().transpose() << "\n"
    //         << " - latestRobotState.getJointVelocity(): " << latestRobotState.getJointVelocity().transpose();
    //     assert(false);
    //     return false;
    // }

    return true;
}

bool RobotInterfaceBenchmark::write(const controlit::Command & command)
{
    // if(commandPublisher && commandPublisher->trylock())
    // {
    //     // Verify dimensions match
    //     assert(commandPublisher->msg_.position.size() == (size_t)command.getPositionCmd().size());

    //     if (!controlit::addons::eigen::checkMagnitude(command.getPositionCmd())
    //         || !controlit::addons::eigen::checkMagnitude(command.getVelocityCmd())
    //         || !controlit::addons::eigen::checkMagnitude(command.getEffortCmd(), 1e3))
    //     {
    //         CONTROLIT_ERROR_RT << "Attempted to write invalid command!\n"
    //             << " - command.getPositionCmd(): " << command.getPositionCmd().transpose() << "\n"
    //             << " - command.getVelocityCmd(): " << command.getVelocityCmd().transpose() << "\n"
    //             << " - command.getEffortCmd(): " << command.getEffortCmd().transpose();
    //         assert(false);
    //         return false;
    //     }

    //     // Set the time stamp
    //     commandPublisher->msg_.header.stamp = time;

    //     // Save the command into the message
    //     for (unsigned int ii = 0; ii < command.getNumDOFs(); ii++)
    //     {
    //         commandPublisher->msg_.position[ii] = command.getPositionCmd()[ii];
    //         commandPublisher->msg_.velocity[ii] = command.getVelocityCmd()[ii];
    //         commandPublisher->msg_.effort[ii] = command.getEffortCmd()[ii];
    //     }

    //     // Set the RTT sequence number
    //     if (rttSeqNo == rcvdRttSeqNo)
    //     {
    //         rttSeqNo++;
    //         commandPublisher->msg_.rtt_seq_no = rttSeqNo;
    //         lastRTTUpdate = high_resolution_clock::now();
    //     }

    //     // Publish the message
    //     commandPublisher->unlockAndPublish();
    // }
    return true;
}

// void RobotInterfaceBenchmark::robotStateCallback(const boost::shared_ptr<controlit_robot_interface_library::JointState const> & robotState)
// {

//     // Check if RTT can be computed
//     if (robotState->rtt_seq_no == rttSeqNo)
//     {
//         rcvdRttSeqNo = rttSeqNo;  // This will trigger the sender to increment the rttSeqNo

//         if(rttLatencyPublisher->trylock())
//         {
//             // Compute the RTT.
//             std::chrono::nanoseconds timeSpan = duration_cast<std::chrono::nanoseconds>(
//                 high_resolution_clock::now() - lastRTTUpdate);

//             // Save the RTT in a message and publish it
//             rttLatencyPublisher->msg_.data = timeSpan.count() / 1e9;
//             rttLatencyPublisher->unlockAndPublish();
//         }
//     }

//     // Save the robot state in the local inactiveRobotState variable
//     *inactiveRobotState = *(robotState.get());

//     // grab lock on mutex, swap the active and inactive robot states, release the lock
//     mutex.lock();

//     std::swap(activeRobotState, inactiveRobotState);

//     // release lock on mutex
//     mutex.unlock();

//     // Set a flag indicating that a robot state was received
//     receivedRobotState = true;
// }

} // namespace robot_interface_library
} // namespace controlit
