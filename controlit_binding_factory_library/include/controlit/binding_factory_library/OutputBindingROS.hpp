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

#ifndef __CONTROLIT_BINDING_FACTORY_LIBRARY_OUTPUT_BINDING_ROS_HPP__
#define __CONTROLIT_BINDING_FACTORY_LIBRARY_OUTPUT_BINDING_ROS_HPP__

#include <ros/ros.h>
#include <controlit/Binding.hpp>
#include <controlit/BindingConfig.hpp>
#include <controlit/addons/ros/RealTimePublisher.hpp>

#include <controlit/logging/RealTimeLogging.hpp>
#include <controlit/addons/ros/ROSMsgEigenConversion.hpp> // for matrixEigenToMsg

// ROS messge types
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#include <visualization_msgs/Marker.h>

#include <mutex>

#include <controlit/addons/eigen/LinearAlgebra.hpp>

namespace controlit {
namespace binding_factory_library {

using controlit::addons::eigen::Vector;
using controlit::addons::eigen::Matrix;
using controlit::addons::eigen::Vector3d;
using controlit::addons::eigen::Matrix3d;

using controlit::addons::ros::matrixEigenToMsg;

/*!
 * Implements an output binding between a parameter and a ROS topic.
 */
template<class DataTypeROS, class DataTypeControlIt>
class OutputBindingROS : public controlit::Binding
{
public:
    /*!
     * The constructor.
     */
    OutputBindingROS(ros::NodeHandle & nh,
        controlit::Parameter * param, const controlit::BindingConfig & config) :
        Binding(param, config), // Call super-class' constructor
        publishRate(100.0)
    {
        // Bind to the parameter's update mechanism
        param->addListener(boost::bind(&OutputBindingROS<DataTypeROS,DataTypeControlIt>::parameterCallback, this, _1));

        // Get queue size if it was specified
        unsigned int queueSize = 1;
        if (config.hasProperty("queue_size"))
        {
            queueSize = config.getTypedProperty<unsigned int>("queue_size");
        }

        // Get publish rate, if it exists. If not, we will cap it at 100 Hz.
        if (config.hasProperty("publish_rate"))
        {
            publishRate = config.getTypedProperty<float>("publish_rate");
        }

        // Get latched option
        bool latched = false;
        if (config.hasProperty("latched"))
        {
            latched = config.getTypedProperty<bool>("latched");
        }

        topic = config.getProperty("topic");

        // Create published. getProperty() will throw an error if it doesn't exist
        //pub.reset(new realtime_tools::RealtimePublisher<DataTypeROS>(nh, config.getProperty("topic"), queueSize, latched));
        publisher.init(nh, topic, queueSize, latched); //.reset(new controlit::addons::ros::RealtimePublisher<DataTypeROS>(nh, topic, queueSize, latched));

        // Initialize the message
        while (!publisher.trylock()) usleep(200);
        initializeMessage(publisher.msg_);
        publisher.unlock();

        // Initialize last publish time to zero
        // This will ensure that we publish the first set() method that comes in.. Important
        // for slowly changing (or never changing) parameters.
        lastPublishTime.fromSec(0.);

        if (latched)
        {
            // If bindings have the latched property set, we want to get the default or constructor
            // value of the parameter out there in the ROS ecosphere. There's a good chance
            // that some output parameters will never call the parameter set() method after the
            // parameter value has been set through its constructor.
            if (publisher.trylock())
            {
                //populateMessage(clock.getSystemTime(ros::Time::now()), *(controlit::ParameterAccessor<DataTypeControlIt>::get(param)), publisher.msg_);
                populateMessage(ros::Time::now(), *(controlit::ParameterAccessor<DataTypeControlIt>::get(param)), publisher.msg_);
                publisher.unlockAndPublish();
            }
        }

        // CONTROLIT_INFO << "Created\n"
        //                << " - topic: " << topic << "\n"
        //                << " - address: " << this << "\n"
        //                << " - parameter name: " << param->name();
    }

    /*!
     * The destructor.
     */
    virtual ~OutputBindingROS()
    {
        // CONTROLIT_INFO << "Destroyed\n"
        //                << " - topic: " << topic << "\n"
        //                << " - address: " << this << "\n"
        //                << " - parameter name: " << param->name();
    }

    /*!
     * Called when the parameter changes.
     *
     * \param[in] param The parameter that changed.
     */
    virtual void parameterCallback(controlit::Parameter const & param)
    {
        /// @todo find a way to get rid of this now()
        ros::Time time = ros::Time::now();

        // limit rate of publishing
        if (lastPublishTime + ros::Duration(1.0/publishRate) < time)
        {
            // try to publish
            if (publisher.trylock())
            {
                // we're actually publishing, so increment time
                lastPublishTime = lastPublishTime + ros::Duration(1.0/publishRate);
                populateMessage(time, *(controlit::ParameterAccessor<DataTypeControlIt>::get(& param)), publisher.msg_);
                PRINT_INFO_STATEMENT("Publishing parameter " << param.name());
                publisher.unlockAndPublish();
            }
            else
            {
                PRINT_INFO_STATEMENT("Unable to obtain lock, not publishing parameter " << param.name());
            }
        }
        else
        {
            PRINT_INFO_STATEMENT("Min publish duration has not elapsed.  Is /clock advancing?");
        }
    }

protected:

    /*!
     * Save the ControlIt! parameter into a ROS message.
     *
     * \param[in] time The current time.
     * \param[in] data The parameter data.
     * \param[out] msg The ROS message in which to save the parameter data.
     */
    virtual void populateMessage(ros::Time const & time, DataTypeControlIt const & data, DataTypeROS & msg)
    {
        // populate joint state message
        publisher.msg_.data = data;
    }

    /*!
     * Initializes the ROS message.
     *
     * \param message The ROS message to initialize.
     */
    virtual void initializeMessage(DataTypeROS & msg)
    {
    }

    /*!
     * The ROS topic publisher.
     */
    controlit::addons::ros::RealtimePublisher<DataTypeROS> publisher;

    /*!
     * The time when this object last published on the ROS topic.  This is used
     * to throttle the rate of publishing.
     */
    ros::Time lastPublishTime;

    /*!
     * The desired publish rate in Hz.
     */
    float publishRate;

    /*!
     * The name of the topic on which to publish.
     */
    std::string topic;
};

// ***************************************************************************
//  Standard message extensions
// ***************************************************************************
template<>
void OutputBindingROS<std_msgs::Float64MultiArray, Vector>::populateMessage(ros::Time const & time,
    Vector const & data, std_msgs::Float64MultiArray & msg)
{
    matrixEigenToMsg(data, msg);
}

template<>
void OutputBindingROS<std_msgs::Float64MultiArray, Matrix>::populateMessage(ros::Time const & time,
    Matrix const & data, std_msgs::Float64MultiArray & msg)
{
    matrixEigenToMsg(data, msg);
}

// ***************************************************************************
//  Geometry message extensions
// ***************************************************************************
// Vector -> Point
void msgcpy(geometry_msgs::Point &msg, Vector const & data)
{
  if (data.rows() == 3)
  {
    msg.x = data(0);
    msg.y = data(1);
    msg.z = data(2);
  }
}

template<>
void OutputBindingROS<geometry_msgs::Point, Vector>::populateMessage(ros::Time const & time,
    Vector const & data, geometry_msgs::Point& msg)
{
  msgcpy(msg, data);
}

template<>
void OutputBindingROS<geometry_msgs::PointStamped, Vector>::initializeMessage(geometry_msgs::PointStamped & msg)
{
  if (config.hasProperty("frame_id"))
    msg.header.frame_id = config.getTypedProperty<std::string>("frame_id");
}

template<>
void OutputBindingROS<geometry_msgs::PointStamped, Vector>::populateMessage(ros::Time const & time,
    Vector const & data, geometry_msgs::PointStamped & msg)
{
    msg.header.stamp = time;
    msgcpy(msg.point, data);
}

// Vector -> Pose
void msgcpy(geometry_msgs::Pose & msg, Vector const & v)
{
    if (v.rows() == 7)
    {
        msg.position.x = v(0);
        msg.position.y = v(1);
        msg.position.z = v(2);

        msg.orientation.w = v(3);
        msg.orientation.x = v(4);
        msg.orientation.y = v(5);
        msg.orientation.z = v(6);
    }
}

template<>
void OutputBindingROS<geometry_msgs::Pose, Vector>::populateMessage(ros::Time const & time, Vector const & data, geometry_msgs::Pose& msg)
{
    msgcpy(msg, data);
}

template<>
void OutputBindingROS<geometry_msgs::PoseStamped, Vector>::initializeMessage(geometry_msgs::PoseStamped& msg)
{
    if (config.hasProperty("frame_id"))
        msg.header.frame_id = config.getTypedProperty<std::string>("frame_id");
}

template<>
void OutputBindingROS<geometry_msgs::PoseStamped, Vector>::populateMessage(ros::Time const & time, Vector const & data, geometry_msgs::PoseStamped& msg)
{
    msg.header.stamp = time;
    msgcpy(msg.pose, data);
}

// Matrix -> Pose
void msgcpy(geometry_msgs::Pose &msg, Matrix const & mat)
{
    if (mat.rows() == 4 and mat.cols() == 4)
    {
        msg.position.x = mat(0,3);
        msg.position.y = mat(1,3);
        msg.position.z = mat(2,3);

        // Create homogeneous transformation matrix
        Eigen::Matrix3d R = mat.block(0,0,3,3);
        Eigen::Quaterniond quat(R);

        msg.orientation.w = quat.w();
        msg.orientation.x = quat.x();
        msg.orientation.y = quat.y();
        msg.orientation.z = quat.z();
    }
}

template<>
void OutputBindingROS<geometry_msgs::Pose, Matrix>::populateMessage(ros::Time const & time, Matrix const & data, geometry_msgs::Pose& msg)
{
    msgcpy(msg, data);
}

template<>
void OutputBindingROS<geometry_msgs::PoseStamped, Matrix>::initializeMessage(geometry_msgs::PoseStamped& msg)
{
    if (config.hasProperty("frame_id"))
        msg.header.frame_id = config.getTypedProperty<std::string>("frame_id");
}

template<>
void OutputBindingROS<geometry_msgs::PoseStamped, Matrix>::populateMessage(ros::Time const & time, Matrix const & data, geometry_msgs::PoseStamped& msg)
{
    msg.header.stamp = time;
    msgcpy(msg.pose, data);
}

// Vector -> Wrench
void msgcpy(geometry_msgs::Wrench &msg, Vector const & wrench)
{
    msg.force.x = wrench(0);
    msg.force.y = wrench(1);
    msg.force.z = wrench(2);
    msg.torque.x = wrench(3);
    msg.torque.y = wrench(4);
    msg.torque.z = wrench(5);
}

template<>
void OutputBindingROS<geometry_msgs::Wrench, Vector>::populateMessage(ros::Time const & time, Vector const & data, geometry_msgs::Wrench& msg)
{
    msgcpy(msg, data);
}

template<>
void OutputBindingROS<geometry_msgs::WrenchStamped, Vector>::initializeMessage(geometry_msgs::WrenchStamped& msg)
{
    if (config.hasProperty("frame_id"))
        msg.header.frame_id = config.getTypedProperty<std::string>("frame_id");
}

template<>
void OutputBindingROS<geometry_msgs::WrenchStamped, Vector>::populateMessage(ros::Time const & time, Vector const & data, geometry_msgs::WrenchStamped& msg)
{
    msg.header.stamp = time;
    msgcpy(msg.wrench, data);
}

// Vector -> Twist
void msgcpy(geometry_msgs::Twist &msg, Vector const & twist)
{
    msg.linear.x = twist(0);
    msg.linear.y = twist(1);
    msg.linear.z = twist(2);
    msg.angular.x = twist(3);
    msg.angular.y = twist(4);
    msg.angular.z = twist(5);
}

template<>
void OutputBindingROS<geometry_msgs::Twist, Vector>::populateMessage(ros::Time const & time, Vector const & data, geometry_msgs::Twist& msg)
{
    msgcpy(msg, data);
}

template<>
void OutputBindingROS<geometry_msgs::TwistStamped, Vector>::initializeMessage(geometry_msgs::TwistStamped& msg)
{
    if (config.hasProperty("frame_id"))
        msg.header.frame_id = config.getTypedProperty<std::string>("frame_id");
}

template<>
void OutputBindingROS<geometry_msgs::TwistStamped, Vector>::populateMessage(ros::Time const & time, Vector const & data, geometry_msgs::TwistStamped& msg)
{
    msg.header.stamp = time;
    msgcpy(msg.twist, data);
}
// ***************************************************************************
//  Visualization message extensions
// ***************************************************************************
template<>
void OutputBindingROS<visualization_msgs::Marker, Vector>::initializeMessage(visualization_msgs::Marker & msg)
{
    if (config.hasProperty("frame_id"))
        msg.header.frame_id = config.getProperty("frame_id");

    if (config.hasProperty("namespace"))
        msg.ns = config.getProperty("namespace");

    if (config.hasProperty("id"))
        msg.id = config.getTypedProperty<int>("id");

    if (config.hasProperty("marker_type"))
        msg.type = config.getTypedProperty<int>("marker_type");

    if (config.hasProperty("scale"))
    {
        std::vector<double> scale = config.getTypedVectorProperty<double>("scale");
        msg.scale.x = scale[0];
        msg.scale.y = scale[1];
        msg.scale.z = scale[2];
    }

    if (config.hasProperty("color"))
    {
        std::vector<double> color = config.getTypedVectorProperty<double>("color");
        msg.color.r = color[0];
        msg.color.g = color[1];
        msg.color.b = color[2];
        msg.color.a = color[3];
    }

    if (config.hasProperty("lifetime"))
    {
        msg.lifetime = ros::Duration(config.getTypedProperty<double>("lifetime"));
    }

    if (config.hasProperty("frame_locked"))
    {
        msg.frame_locked = config.getTypedProperty<int>("frame_locked");
    }
}

template<>
void OutputBindingROS<visualization_msgs::Marker, Vector>::populateMessage(ros::Time const & time,
    Vector const & data, visualization_msgs::Marker & msg)
{
    msg.header.stamp = time;
    msgcpy(msg.pose, data);
}

} // namespace binding_factory_library
} // namespace controlit

#endif // __CONTROLIT_BINDING_FACTORY_LIBRARY_OUTPUT_BINDING_ROS_HPP__
