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

#ifndef __CONTROLIT_CORE_ROBOT_INTERFACE_HPP__
#define __CONTROLIT_CORE_ROBOT_INTERFACE_HPP__

#include "ros/ros.h"
#include <std_msgs/Float64.h>

#include <controlit/Timer.hpp>
#include <controlit/OdometryStateReceiver.hpp>
#include <controlit/addons/ros/RealTimePublisherHeader.hpp>

namespace controlit {

// Forward declarations
class RTControlModel;
class RobotState;
class Command;

/*!
 * The top-level definition of a ControlIt! robot interface.  This class specifies the
 * methods that all robot interface clases must implement.
 */
class RobotInterface
{
public:
    /*!
     * The constructor.
     */
    RobotInterface();

    /*!
     * The destructor.
     */
    virtual ~RobotInterface();

    /*!
     * Initializes this robot interface.
     *
     * \param[in] nh The ROS node handle to use during the initialization
     * process.
     * \param[in] model The robot model.
     * \return Whether the initialization was successful.
     */
    virtual bool init(ros::NodeHandle & nh, RTControlModel * model);

    /*!
     * Obtains the current state of the robot.
     *
     * \param[out] latestRobotState The variable in which to store the latest robot state.
     * \param[in] block Whether to block waiting for message to arrive.
     * \return Whether the read was successful.
     */
    virtual bool read(RobotState & latestRobotState, bool block = false);

    /*!
     * Sends a command to the robot.
     *
     * \param[in] time The current time.
     * \param[in] command The command to send to the robot.
     * \return Whether the write was successful.
     */
    virtual bool write(const Command & command);

    /*!
     * Returns a timer. The default implementation uses TimerROS.
     * However, child classes may override this method with platform-specific
     * ways of obtaining the current time.
     */
    virtual std::shared_ptr<Timer> getTimer();

protected:

    /*!
     * Publishes the round trip communication latency.
     *
     * \param[in] latency The round trip communication latency between ControlIt!
     * and the robot.
     */
    void publishCommLatency(double latency);

    /*!
     * Real-time safe publisher for the round trip communication latency.
     */
    controlit::addons::ros::RealtimePublisher<std_msgs::Float64>
        rttLatencyPublisher;

    /*!
     * Whether the controller is initialized.
     */
    bool initialized;

    /*!
     * The name of the controller.  This is useful for creating a ROS node handle.
     */
    std::string controllerName;

    /*!
     * The robot control model.
     */
    RTControlModel * model;

    /*!
     * For receiving Odometry information
     */
    std::unique_ptr<OdometryStateReceiver> odometryStateReceiver;

    //---------------------------------------------------------------------------------
    // The following variables are used to compute the round trip communication latency
    // between ControlIt! and the robot.
    //---------------------------------------------------------------------------------

    /*!
     * The current sequence number being transmitted with the command. This is used to measure the
     * round-trip communication latency between ControlIt! and the robot.
     */
    int64_t seqno;
    
    /*!
     * Whether to send the next sequence number.
     */
    bool sendSeqno;

    /*!
     * A timer for measuring the round trip communication time.
     */
    std::shared_ptr<Timer> rttTimer;

    //---------------------------------------------------------------------------------
    // The following variables are used to publish the latest command and joint state
    // onto ROS topics.
    //---------------------------------------------------------------------------------

    /*!
     * This is used to throttle the publishing of the command and state messages.
     * It is incremented each time read is called and these messages are only
     * published when publishCounter is divisible by 10.
     */
    int publishCounter;

    /*!
     * Publishes the command issued by ControlIt!
     */
    controlit::addons::ros::RealtimePublisherHeader<sensor_msgs::JointState>
        commandPublisher;

    /*!
     * Publishes the robot state.
     */
    controlit::addons::ros::RealtimePublisherHeader<sensor_msgs::JointState>
        statePublisher;

private:
    bool useROSTimer;

};

} // namespace controlit

#endif // __CONTROLIT_CORE_ROBOT_INTERFACE_HPP__
