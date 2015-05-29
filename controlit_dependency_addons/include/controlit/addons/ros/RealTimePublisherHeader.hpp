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

/*
 * Extends RealTimePublisher to set the timestamp in the message's header.
 * It is only compatible with ROS messages that contain headers.
 *
 * Original author: Chien-Liang Fok
 */
#ifndef __CONTROLIT_ADDONS_ROS_REALTIME_PUBLISHER_HEADER_HPP__
#define __CONTROLIT_ADDONS_ROS_REALTIME_PUBLISHER_HEADER_HPP__

#include <controlit/addons/ros/RealTimePublisher.hpp>
#include "ros/ros.h" // for ros::Time

namespace controlit {
namespace addons {
namespace ros {

template <class Msg>
class RealtimePublisherHeader : public RealtimePublisher<Msg>
{
public:

    /**  \brief Constructor for the realtime publisher
     *
     * \param topic the topic name to advertise
     * \param queue_size the size of the outgoing ROS buffer
     * \param latched . optional argument (defaults to false) to specify is publisher is latched or not
     */
    RealtimePublisherHeader(const std::string &topic, int queue_size, bool latched=false) :
        RealtimePublisher<Msg>(topic, queue_size, latched)
    {
    }

    /**  \brief Constructor for the realtime publisher
     *
     * \param node the nodehandle that specifies the namespace (or prefix) that is used to advertise the ROS topic
     * \param topic the topic name to advertise
     * \param queue_size the size of the outgoing ROS buffer
     * \param latched . optional argument (defaults to false) to specify is publisher is latched or not
     */
    // RealtimePublisherHeader(const ::ros::NodeHandle & node, const std::string &topic, int queue_size, bool latched=false) :
    //     RealtimePublisher<Msg>(node, topic, queue_size, latched)
    // {
    // }

    RealtimePublisherHeader()
    {
    }

    /// Destructor
    ~RealtimePublisherHeader()
    {
    }

protected:

    virtual void publish(Msg msg)
    {
        msg.header.stamp = ::ros::Time::now();   
        RealtimePublisher<Msg>::publish(msg);
    }
};

} // namespace ros
} // namespace addons
} // namespace controlit

#endif  //__CONTROLIT_ADDONS_ROS_REALTIME_PUBLISHER_HEADER_HPP__
