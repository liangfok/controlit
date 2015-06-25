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

#ifndef __CONTROLIT_GRAVITY_VECTOR_PUBLISHER_HPP__
#define __CONTROLIT_GRAVITY_VECTOR_PUBLISHER_HPP__

#include <vector>
#include "ros/ros.h"

#include <controlit/addons/eigen/LinearAlgebra.hpp>
#include <controlit/addons/ros/RealTimePublisherHeader.hpp>
#include <sensor_msgs/JointState.h>

namespace controlit {
namespace utility {

using controlit::addons::eigen::Vector;

/*!
 * Publishes the gravity compensation vector as a
 * sensor_msgs::JointState message.
 */
class GravityCompensationPublisher
{
public:
    /*!
     * The constructor.
     */
    GravityCompensationPublisher();

    /*!
     * The destructor.
     */
    ~GravityCompensationPublisher() {}

    /*!
     * Initializes this class by loading the parameters from the ROS
     * parameter server.
     *
     * \param[in] nh The ROS node handle with which to work with.
     * \param[in] actuatedJointNames The names of the joints to be included
     * in published gravity compensation vector.
     */
    bool init(ros::NodeHandle & nh, const std::vector<std::string> & actuatedJointNames);

    /*!
     * Publish the gravity compensation vector.
     *
     * \param[in] gravCompVec The gravity compensation vector.  The order
     * of the elements in this vector must match the order of the joint names
     * passed into the init() method.
     * \param[in] jointPositions The current joint positions (both real and virtual).
     * \param[in] jointVelocities The current joint velocities (both real and virtual).
     * \return Whether the gravity compensation vetor was successfully
     * published.
     */
    bool publish(const Vector & gravCompVec,
        const Vector & jointPositions, const Vector & jointVelocities);

private:

    /*!
     * Remembers the number of times the publish method was called.
     */
    size_t callCounter;

    /*!
     * This is the publisher.
     */
    controlit::addons::ros::RealtimePublisherHeader<sensor_msgs::JointState> publisher;
};

} // namespace utility
} // namespace controlit

#endif
