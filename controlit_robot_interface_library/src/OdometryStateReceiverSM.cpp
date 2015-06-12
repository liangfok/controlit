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

#include <controlit/robot_interface_library/OdometryStateReceiverSM.hpp>

#include <thread>  // for std::thread::sleep_for(...)
#include <unistd.h>

namespace controlit {
namespace robot_interface_library {

// Uncomment one of the following lines to enable/disable detailed debug statements.
#define PRINT_DEBUG_STATEMENT(ss)
// #define PRINT_DEBUG_STATEMENT(ss) CONTROLIT_DEBUG << ss;

#define PRINT_DEBUG_STATEMENT_RT(ss)
// #define PRINT_DEBUG_STATEMENT_RT(ss) CONTROLIT_DEBUG_RT << ss;

#define PRINT_INFO_STATEMENT_RT_ALWAYS(ss) CONTROLIT_INFO_RT << ss;
// #define PRINT_INFO_STATEMENT_RT_ALWAYS(ss) std::cout << ss << std::endl;

// Parameters for shared memory subscribers
#define LISTEN_TO_ROS_TOPIC false
#define USE_POLLING false

OdometryStateReceiverSM::OdometryStateReceiverSM() :
    OdometryStateReceiver(), // Call super-class' constructor
    
    cachedOdometryData(new OdometryData()), // Initialize odometry data to arbitrary default values.
                                            // There is a boolean variable called "receivedOdometryState"
                                            // that prevents this variable from being used by the
                                            // controller until after real odometry data is received.
    odometrySubscriber(LISTEN_TO_ROS_TOPIC, USE_POLLING),
    isFirstReception(true)
{
}

OdometryStateReceiverSM::~OdometryStateReceiverSM()
{
    if (cachedOdometryData != nullptr) delete cachedOdometryData;
}

bool OdometryStateReceiverSM::init(ros::NodeHandle & nh, RTControlModel * model)
{
    // Initialize the parent class.
    bool initSuccess = OdometryStateReceiver::init(nh, model);

    // If the super-class fails to initialize, abort.
    if (!initSuccess) return false;

    // Obtain the topics for obtaining the odometry data
    if (!nh.getParam("controlit/odometry_topic", odometryTopic))
        odometryTopic = "/robot_base_state";

    // Initialize the odometryDataBuffer.
    odometryDataBuffer.initRT(*cachedOdometryData);

    PRINT_DEBUG_STATEMENT("Subscribing to odometry topic...");
    odometrySubscriber.subscribe(odometryTopic, boost::bind(&OdometryStateReceiverSM::odometryMessageCallback, this, _1));
    
    // Wait for message to arrive to ensure connection 
    // is initialized prior to starting the controller.
    // nav_msgs::Odometry tmpMsg;
    // odometrySubscriber.waitForMessage(tmpMsg);
    
    return true;
}

bool OdometryStateReceiverSM::getOdometry(controlit::RobotState & latestRobotState, bool block)
{
    // If we haven't received robot state yet, either return fail or spin wait for
    // the state to arrive.
    if (!receivedOdometryState)
    {
        if (!block)
        {
            if (++numConsecutiveFailures > MAX_NUM_FAILURES_BEFORE_WARNING)
            {
                CONTROLIT_WARN_RT << "Did not receive odometry state, aborting.";
                numConsecutiveFailures = 0; // reset
            }
            return false;
        }
        else
        {
            // Wait until message is received
            while (!receivedOdometryState)
            {
                #define WAIT_DURATION_MS 100
                PRINT_DEBUG_STATEMENT("Waiting " << WAIT_DURATION_MS << " ms for odometry state message.")
                std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_DURATION_MS));
            }
        }
    }

    numConsecutiveFailures = 0; // reset

    // Save the odometry data
    *cachedOdometryData = *(odometryDataBuffer.readFromRT());
    if (!latestRobotState.setRobotBaseState(cachedOdometryData->x, cachedOdometryData->q, cachedOdometryData->x_dot))
    {
        CONTROLIT_WARN_RT << "Failed to set robot base state, aborting this read operation.";
        return false;
    }

    return true;
}

// NOTE: Odometry data is used to set the 6 virtual degrees of freedom.
void OdometryStateReceiverSM::odometryMessageCallback(nav_msgs::Odometry & odom)
{
    // CONTROLIT_INFO_RT << "Method called!";
    if (isFirstReception)
    {
        CONTROLIT_INFO_RT << "Received initial odometry message.";
        isFirstReception = false;
    }
    else
    {
        // Assuming odometry information in the correct frame...
        Vector x(3);
        x(0) = odom.pose.pose.position.x;
        x(1) = odom.pose.pose.position.y;
        x(2) = odom.pose.pose.position.z;

        // Get the orientation
        Eigen::Quaterniond q(odom.pose.pose.orientation.w,
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z);

        // Get the velocity
        Vector x_dot(6);
        x_dot(0) = odom.twist.twist.linear.x;
        x_dot(1) = odom.twist.twist.linear.y;
        x_dot(2) = odom.twist.twist.linear.z;
        x_dot(3) = odom.twist.twist.angular.x;
        x_dot(4) = odom.twist.twist.angular.y;
        x_dot(5) = odom.twist.twist.angular.z;

        if (!controlit::addons::eigen::checkMagnitude(x) || !controlit::addons::eigen::checkMagnitude(q) || !controlit::addons::eigen::checkMagnitude(x_dot))
        {
            CONTROLIT_ERROR_RT << "Received invalid odometry data!\n"
                << "  - x: " << x.transpose() << "\n"
                << "  - q: [" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << "]\n"
                << "  - x_dot: " << x_dot.transpose();
            assert(false);
        }
        else
        {
            // the writeFromNonRT can be used in RT, if you have the guarantee that
            //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
            //  * there is only one single rt thread
            odometryDataBuffer.writeFromNonRT( OdometryData(x, q, x_dot) );
            receivedOdometryState = true;
        }
    }
}

} // namespace robot_interface_library
} // namespace controlit
