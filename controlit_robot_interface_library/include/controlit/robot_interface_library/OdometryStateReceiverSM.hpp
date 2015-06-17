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

#ifndef __CONTROLIT_ROBOT_INTERFACE_LIBRARY_ODOMETRY_STATE_RECEIVER_SM_HPP__
#define __CONTROLIT_ROBOT_INTERFACE_LIBRARY_ODOMETRY_STATE_RECEIVER_SM_HPP__

#include <ros/ros.h>  // for ROS Time
#include <controlit/RobotState.hpp>
#include <controlit/OdometryData.hpp>

#include <nav_msgs/Odometry.h>

#include <controlit/OdometryStateReceiver.hpp>
#include <controlit/EigenRealtimeBuffer.hpp>

#include <shared_memory_interface/shared_memory_subscriber.hpp>

namespace controlit {
namespace robot_interface_library {

/*!
 * Supplies odometry information based on shared memory.
 */
class OdometryStateReceiverSM : public OdometryStateReceiver
{
public:
    /*!
     * The constructor.
     */
    OdometryStateReceiverSM();

    /*!
     * The destructor.
     */
    ~OdometryStateReceiverSM();

    /*!
     * Initializes this class.
     *
     * \param[in] nh The ROS node handle to use during the initialization
     * process.
     * \param[in] model The robot model.
     * \return Whether the initialization was successful.
     */
    virtual bool init(ros::NodeHandle & nh, RTControlModel * model);

    /*!
     * Obtains the current odometry state.
     *
     * \param[out] latestRobotState This is where the robot's odometry state is stored.
     * \param[in] block Whether to block waiting for the odometry state to arrive.
     * \return Whether the odometry state was successfully obtained.
     */
    virtual bool getOdometry(controlit::RobotState & latestRobotState, bool block = false);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:

    /*!
     * The callback function for the subscription to odometry messages.
     * It stores the odometry information in the odometryDataBuffer so that
     * it can be accessed in a real-time-safe manner.
     *
     * \param odom A message containing the odometry state.
     */
    void odometryMessageCallback(nav_msgs::Odometry& odom);

    /*!
     * Cached odometry data.. to avoid resizing Eigen vectors at runtime.
     */
    OdometryData * cachedOdometryData;

    /*!
     * A buffer for holding the most recently recieved odometry state.
     * This is useful for real-time-safe access to this data.
     */
    EigenRealtimeBuffer<OdometryData> odometryDataBuffer;

    /*!
     * The shared memory interface.
     */
    shared_memory_interface::Subscriber<nav_msgs::Odometry> odometrySubscriber;

    /*!
     * The topic on which to receive odometry information.
     */
    std::string odometryTopic;

    bool rcvdInitOdomMsg;
    // bool rcvdFirstOdomMsg;
};

} // namespace robot_interface_library
} // namespace controlit

#endif  // __CONTROLIT_ROBOT_INTERFACE_LIBRARY_ODOMETRY_STATE_RECEIVER_SM_HPP__
