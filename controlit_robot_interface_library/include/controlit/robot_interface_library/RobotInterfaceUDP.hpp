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

#ifndef __CONTROLIT_ROBOT_INTERFACE_LIBRARY_ROBOT_INTERFACE_UDP_HPP__
#define __CONTROLIT_ROBOT_INTERFACE_LIBRARY_ROBOT_INTERFACE_UDP_HPP__

#include <controlit/RobotInterface.hpp>
// #include <controlit/robot_interface_library/comm_udp.h>
#include <controlit_udp/TxRxUDP.hpp>
#include <controlit/addons/ros/RealTimePublisher.hpp>
#include <sensor_msgs/JointState.h>

namespace controlit {
namespace robot_interface_library {

/*!
 * The robot interface for robots that are controlled via UDP datagrams.
 */
class RobotInterfaceUDP : public controlit::RobotInterface
{
public:
    /*!
     * The constructor.
     */
    RobotInterfaceUDP();

    /*!
     * The destructor.
     */
    virtual ~RobotInterfaceUDP();

    /*!
     * Initializes this robot interface.
     *
     * \param[in] nh The ROS node handle to use during the initialization
     * process.
     * \param[in] model The robot model.
     * \return Whether the initialization was successful.
     */
    virtual bool init(ros::NodeHandle & nh, RTControlModel * model);


    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:

    /*!
     * Obtains the current state of the robot.
     *
     * \param[out] latestRobotState The variable in which to store the
     * latest robot state.
     * \param[in] block Whether to block waiting for message to arrive.
     * \return Whether the read was successful.
     */
    virtual bool read(controlit::RobotState & latestRobotState, bool block = false);

    /*!
     * Sends a command to the robot.
     *
     * \param[in] command The command to send to the robot.
     * \return Whether the write was successful.
     */
    virtual bool write(const controlit::Command & command);

private:
    /*!
     * UDP adapter for tx/rx robot state and commands
     */
    controlit_udp::TxCommandRxStateUDP UDP;

    /*!
     * Holds received messages.
     */
    controlit_udp::StateMsg rsMsg;

    /*!
     * Holds the command.
     */
    controlit_udp::CommandMsg cmdMsg;

    /*!
     * Publishes the command.
     */
    // std::unique_ptr<controlit::addons::ros::RealtimePublisher<sensor_msgs::JointState>>
    //     commandPublisher;

    /*!
     * Publishes the robot state.
     */
    // std::unique_ptr<controlit::addons::ros::RealtimePublisher<sensor_msgs::JointState>>
    //     statePublisher;

};

} // namespace robot_interface_library
} // namespace controlit

#endif // __CONTROLIT_ROBOT_INTERFACE_LIBRARY_ROBOT_INTERFACE_UDP_HPP__
