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

#ifndef __CONTROLIT_ROBOT_INTERFACE_LIBRARY_ROBOT_INTERFACE_SM_HPP__
#define __CONTROLIT_ROBOT_INTERFACE_LIBRARY_ROBOT_INTERFACE_SM_HPP__

#include <controlit/addons/ros/RealTimePublisher.hpp>
#include <controlit/RobotInterface.hpp>
#include <thread>  // for std::mutex
#include <unistd.h>
#include <controlit_robot_interface_library/JointState.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <urdf/model.h>

#include <shared_memory_interface/shared_memory_publisher.hpp>
#include <shared_memory_interface/shared_memory_subscriber.hpp>

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;

namespace controlit {
namespace robot_interface_library {

/*!
 * The coordinator for robots that are controlled via ROS topics.
 *
 * The robot_hardware_interface::ImpedanceCommandInterface type is
 * used as the template type for the Coordinator parent class but is
 * not actually used by this class.
 */
class RobotInterfaceSM : public controlit::RobotInterface
{
public:
    /*!
     * The constructor.
     */
    RobotInterfaceSM();

    /*!
     * The destructor.
     */
    virtual ~RobotInterfaceSM();

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
     * The callback method for receiving reflected RTT (round trip time) sequence numbers from the robot.
     */
    void rttCallback(std_msgs::Int64 & msg);


    /*!
     * The callback method for receiving joint state information from the robot.
     */
    void jointStateCallback(sensor_msgs::JointState & msg);
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
     * Whether a robot state message was received.
     */
    bool receivedRobotState;

    /*!
     * Writes a test command into shared memory.  The command
     * has a recognizable pattern.  Useful for debugging purposes.
     */
    void writeTestCmd();

    /*!
     * Initializes the shared memory joint name to index map.
     *
     * \return true if successful.  false otherwise.
     */
    bool loadSMJointNameToIndexMap();

    /*!
     * Pointer to the handle that subscribes to robot joint state messages from
     * the shared memory.
     */
    shared_memory_interface::Subscriber<sensor_msgs::JointState> robotStateSubscriber;

    /*!
     * Pointer to the handle that subscribes to RTT RX messages from
     * the shared memory.
     */
    shared_memory_interface::Subscriber<std_msgs::Int64> rttRxSubscriber;

    /*!
     * Pointer to the handle that publishes command messages to shared memory.
     */
    shared_memory_interface::Publisher<std_msgs::Float64MultiArray> cmdPublisher;

    /*!
     * Pointer to the handle that publishes RTT TX messages to shared memory.
     */
    shared_memory_interface::Publisher<std_msgs::Int64> rttTxPublisher;

    /*!
     * Maps the name of a joint to its index within the shared memory.
     */
    std::map<std::string, int> jointNameMap;

    /*!
     * For holding the incoming joint state.
     */
    sensor_msgs::JointState jointStateMsg;

    /*!
     * Ensures only one thread can access jointStateMsg at a time.
     */
    std::mutex jointStateMutex;

    /*!
     * For holding the incoming RTT RX message.  This is used to compute the round-trip communication time.
     */
    std_msgs::Int64 rttRxMsg;

    /*!
     * Ensures only one thread can access rttRxMsg at a time.
     */
    std::mutex rttRxMsgMutex;

    /*!
     * Whether joint state was ever received.
     */
    bool rcvdJointState;

    /*!
     * The message containing the torque command.
     */
    std_msgs::Float64MultiArray torqueCmdMsg;

    /*!
     * The message containing the current RTT.  To be transmitted to the robot.
     */
    std_msgs::Int64 currentRTTMsg;

    /*!
     * Whether the initial state message was received.
     */
    bool isFirstReceptionState;

    /*!
     * Whether the initial RTT message was received.
     */
    bool isFirstReceptionRTT;
};

} // namespace robot_interface_library
} // namespace controlit

#endif // __CONTROLIT_ROBOT_INTERFACE_LIBRARY_ROBOT_INTERFACE_SM_HPP__
