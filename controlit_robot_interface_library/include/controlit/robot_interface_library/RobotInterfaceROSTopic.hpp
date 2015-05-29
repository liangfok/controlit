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

#ifndef __CONTROLIT_ROBOT_INTERFACE_LIBRARY_ROBOT_INTERFACE_ROS_TOPIC_HPP__
#define __CONTROLIT_ROBOT_INTERFACE_LIBRARY_ROBOT_INTERFACE_ROS_TOPIC_HPP__

#include <controlit/RobotInterface.hpp>
#include <controlit_robot_interface_library/JointState.h>
#include <controlit/addons/ros/RealTimePublisherHeader.hpp>
#include <thread>  // for std::mutex

#include <controlit/OdometryData.hpp>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

// #include <controlit/addons/ros/ROSParameterAccessor.hpp>

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
// using controlit::addons::ros::ROSParameterAccessor;

namespace controlit {
namespace robot_interface_library {

/*!
 * The coordinator for robots that are controlled via ROS topics.
 *
 * The robot_hardware_interface::ImpedanceCommandInterface type is
 * used as the template type for the Coordinator parent class but is
 * not actually used by this class.
 */
class RobotInterfaceROSTopic : public controlit::RobotInterface
{
public:
    /*!
     * The constructor.
     */
    RobotInterfaceROSTopic();

    /*!
     * The destructor.
     */
    virtual ~RobotInterfaceROSTopic();

    /*!
     * Initializes this robot interface.
     *
     * \param[in] nh The ROS node handle to use during the initialization
     * process.
     * \param[in] model The robot model.
     * \return Whether the initialization was successful.
     */
    virtual bool init(ros::NodeHandle & nh, RTControlModel * model);

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
     * The robot state callback function.
     *
     * \param robotState The message containing the robot state.
     */
    void robotStateCallback(const boost::shared_ptr<controlit_robot_interface_library::JointState const> & robotState);

    /*!
     * This is used to fetch parameters from the ROS parameter server.
     */
    // std::unique_ptr<ROSParameterAccessor> paramAccessor;

    /*!
     * Whether a robot state message was received.
     */
    bool receivedRobotState;

    /*!
     * The robot state subscriber.
     */
    ros::Subscriber robotStateSubscriber;

    /*!
     * For temporarily holding the incoming robot's state.
     * Active and inactive versions are used to minimize the
     * time the mutex is locked.  The active state is used by the RT thread
     * during read().  The inactive state is used by the ROS topic callback thread.
     * The mutex is only locked when swapping the active and inactive pointers.
     */
    controlit_robot_interface_library::JointState robotState1;
    controlit_robot_interface_library::JointState robotState2;
    controlit_robot_interface_library::JointState * activeRobotState;
    controlit_robot_interface_library::JointState * inactiveRobotState;

    /*!
     * Used to publish the command.
     */
    controlit::addons::ros::RealtimePublisherHeader<controlit_robot_interface_library::JointState>
        commandPublisher;

    /*!
     * This mutex ensures no race condition between the ROS topic callback thread
     * and the RT thread.
     */
    std::mutex mutex;
};

} // namespace robot_interface_library
} // namespace controlit

#endif // __CONTROLIT_ROBOT_INTERFACE_LIBRARY_ROBOT_INTERFACE_ROS_TOPIC_HPP__
