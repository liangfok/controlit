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
 * Inspired by realtime_tools::RealtimePublisher.
 * Original author: Stuart Glaser
 * Modified by: JD Yamokoski
 * Modified by: Chien-Liang Fok
 */
#ifndef __CONTROLIT_ADDONS_ROS_REALTIME_PUBLISHER_HPP__
#define __CONTROLIT_ADDONS_ROS_REALTIME_PUBLISHER_HPP__

#include <string>
#include <typeinfo>  // for use of typeid()
#include <boost/utility.hpp>
#include <ros/node_handle.h>

#include <sensor_msgs/JointState.h>

#include <controlit/addons/cpp/GlobalThreadPool.hpp>

namespace controlit {
namespace addons {
namespace ros {

template <class Msg>
class RealtimePublisher : boost::noncopyable
{
public:
    /// The msg_ variable contains the data that will get published on the ROS topic.
    Msg msg_;


    /**  \brief Constructor for the realtime publisher
     *
     * \param node the nodehandle that specifies the namespace (or prefix) that is used to advertise the ROS topic
     * \param topic the topic name to advertise
     * \param queue_size the size of the outgoing ROS buffer
     * \param latched . optional argument (defaults to false) to specify is publisher is latched or not
     */
    RealtimePublisher(const std::string &topic, int queue_size, bool latched=false) :
        topic_(topic)
    {
        construct(queue_size, latched);
    }

    /**  \brief Constructor for the realtime publisher
     *
     * \param node the nodehandle that specifies the namespace (or prefix) that is used to advertise the ROS topic
     * \param topic the topic name to advertise
     * \param queue_size the size of the outgoing ROS buffer
     * \param latched . optional argument (defaults to false) to specify is publisher is latched or not
     */
    // RealtimePublisher(const ::ros::NodeHandle & node, const std::string &topic, int queue_size, bool latched=false) :
    //     topic_(topic),
    //     node_(node)
    // {
    //     construct(queue_size, latched);
    // }

    RealtimePublisher()
    {
    }

    /// Destructor
    ~RealtimePublisher()
    {
        publisher_.shutdown();
    }

    void init(const std::string &topic, int queue_size, bool latched=false)
    {
        topic_ = topic;
        construct(queue_size, latched);
    }

    void init(const ::ros::NodeHandle &node, const std::string &topic, int queue_size, bool latched=false)
    {
        topic_ = topic;
        node_ = node;
        construct(queue_size, latched);
    }

    /**  \brief Try to get the data lock from realtime
     *
     * To publish data from the realtime loop, you need to run trylock to
     * attempt to get unique access to the msg_ variable. Trylock returns
     * true if the lock was aquired, and false if it failed to get the lock.
     */
    bool trylock()
    {
        return controlit::addons::cpp::trylock_thread_pool();
    }

    /**  \brief Unlock the msg_ variable
     *
     * After a successful trylock and after the data is written to the mgs_
     * variable, the lock has to be released for the message to get
     * published on the specified topic.
     */
    void unlockAndPublish()
    {
        controlit::addons::cpp::queue_and_unlock_thread_pool( std::bind(&RealtimePublisher::publish, this, msg_) );
    }

    /**  \brief Get the data lock form non-realtime
     *
     * To publish data from the realtime loop, you need to run trylock to
     * attempt to get unique access to the msg_ variable. Trylock returns
     * true if the lock was aquired, and false if it failed to get the lock.
     */
    void lock()
    {
        controlit::addons::cpp::lock_thread_pool();
    }

    /**  \brief Unlocks the data without publishing anything
     *
     */
    void unlock()
    {
        controlit::addons::cpp::unlock_thread_pool();
    }

protected:
    virtual void publish(Msg msg)
    {
        publisher_.publish(msg);
    }

private:
    void construct(int queue_size, bool latched=false)
    {
        publisher_ = node_.advertise<Msg>(topic_, queue_size, latched);
    }

    std::string topic_;
    ::ros::NodeHandle node_;
    ::ros::Publisher publisher_;
};

} // namespace ros
} // namespace addons
} // namespace controlit
#endif
