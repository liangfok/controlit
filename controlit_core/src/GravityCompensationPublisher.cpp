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

#include <controlit/utility/GravityCompensationPublisher.hpp>
#include <controlit/logging/RealTimeLogging.hpp>

namespace controlit {
namespace utility {

// publish gravity compensation vector once every this number of times.
#define PUBLISH_THROTTLE_FACTOR 100

GravityCompensationPublisher::GravityCompensationPublisher() :
    callCounter(0),
    publisher("diagnostics/gravityCompensationVector", 1)
{
}

bool GravityCompensationPublisher::init(ros::NodeHandle & nh,
    const std::vector<std::string> & actuatedJointNames)
{
    // Create a real-time publisher of the gravity compensation vector
    // publisher.reset(
    //     new controlit::addons::ros::RealtimePublisherHeader<sensor_msgs::JointState>(nh, 
    //         "diagnostics/gravityCompensationVector", 1));

    if(publisher.trylock())
    {
        size_t numJoints = actuatedJointNames.size();
  
        // Initialize the message containing the gravity compensation vector
        publisher.msg_.name.clear();
        publisher.msg_.position.resize(numJoints, 0.0);
        publisher.msg_.velocity.resize(numJoints, 0.0);
        publisher.msg_.effort.resize(numJoints, 0.0);
      
        // Store the joint names
        for (auto & jointName : actuatedJointNames)
        {
            publisher.msg_.name.push_back(jointName);
        }

        publisher.unlockAndPublish();
        return true;
    }
    else
    {
        CONTROLIT_ERROR_RT << "Unable to initialize publisher!";
        return false;
    }
}

bool GravityCompensationPublisher::publish(const Vector & gravCompVec,
    const Vector & jointPositions, const Vector & jointVelocities)
{
    // Only publish once every PUBLISH_THROTTLE_FACTOR times
    if (callCounter++ % PUBLISH_THROTTLE_FACTOR != 0)
        return true;
    
    if(publisher.trylock())
    {

        // Save the data in the message
        for (int ii = 0; ii < gravCompVec.size(); ii++)
        {
            publisher.msg_.effort[ii] = gravCompVec[ii];
            publisher.msg_.position[ii] = jointPositions[ii];
            publisher.msg_.velocity[ii] = jointVelocities[ii];
        }
      
        publisher.unlockAndPublish();
    } 
    else
        callCounter--;

    return true;
}

} // namespace utility
} // namespace controlit
