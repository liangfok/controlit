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

#ifndef __CONTROLIT_CORE_ODOMETRY_STATE_RECEIVER_HPP__
#define __CONTROLIT_CORE_ODOMETRY_STATE_RECEIVER_HPP__

#include <ros/ros.h>  // for ROS Time
#include <controlit/RobotState.hpp>
#include <controlit/OdometryData.hpp>

#include <controlit/RTControlModel.hpp>
#include <controlit/addons/ros/ROSParameterAccessor.hpp>

using controlit::addons::ros::ROSParameterAccessor;

namespace controlit {

#define ODOMETRY_INIT_TIMEOUT 5
#define MAX_NUM_FAILURES_BEFORE_WARNING 3000

/*!
 * This is the super-class of all odometry state receivers.
 * Odometry state receivers are used by RobotInterface objects
 * to set the robot's base state (i.e., its virtual 6 DoFs)
 */
class OdometryStateReceiver
{
public:
    /*!
     * The constructor.
     */
    OdometryStateReceiver();

    /*!
     * The destructor.
     */
    ~OdometryStateReceiver();

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
     * Obtains the current odometry state. The odometry state is stored within
     * the supplied controlit::RobotState object.
     *
     * \param[out] latestRobotState The variable in which to store the
     * robot's odometry state.
     * \param[in] block Whether to block waiting for the odometry state to arrive.
     * \return Whether the odometry state was successfully obtained.
     */
    virtual bool getOdometry(controlit::RobotState & latestRobotState, bool block = false) = 0;

protected:
    /*!
     * Whether odometry state was received.
     */
    bool receivedOdometryState;

    /*!
     * This is used to fetch parameters from the ROS parameter server.
     */
    std::unique_ptr<ROSParameterAccessor> paramAccessor;

    /*!
     * The number of times getOdometry() returns false consecutively.
     * This is used to supress warning messages during initialization.
     */
    int numConsecutiveFailures;
};

} // namespace controlit

#endif  // __CONTROLIT_CORE_ODOMETRY_STATE_RECEIVER_HPP__
