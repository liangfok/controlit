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

#ifndef __CONTROLIT_BINDING_FACTORY_LIBRARY_INPUT_BINDING_ROS_HPP__
#define __CONTROLIT_BINDING_FACTORY_LIBRARY_INPUT_BINDING_ROS_HPP__

#include <controlit/logging/RealTimeLogging.hpp>
#include <controlit/addons/cpp/StringUtilities.hpp>
#include <controlit/addons/ros/ROSMsgEigenConversion.hpp> // for matrixMsgToEigen

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

#include <sensor_msgs/JointState.h>
#include <cctype>

#include <visualization_msgs/Marker.h>

namespace controlit {
namespace binding_factory_library {

using controlit::addons::ros::matrixMsgToEigen;

// Uncomment one of the following lines to enable/disable detailed debug statements.
#define PRINT_INFO_STATEMENT(ss)
// #define PRINT_INFO_STATEMENT(ss) CONTROLIT_INFO << ss;

#define PRINT_INFO_STATEMENT_RT(ss)
// #define PRINT_INFO_STATEMENT_RT(ss) CONTROLIT_DEBUG_RT << ss;

#define PRINT_INFO_STATEMENT_RT_ALWAYS(ss) CONTROLIT_INFO_RT << ss;
// #define PRINT_INFO_STATEMENT_RT_ALWAYS(ss) std::cout << ss << std::endl;

/*!
 * An input binding between a ControlIt! parameter and a ROS topic.
 */
template<class DataTypeROS>
class InputBindingROS : public controlit::Binding
{
public:
    /*!
     * The constructor.
     */
    InputBindingROS(ros::NodeHandle & nh, controlit::Parameter * param, const controlit::BindingConfig & config) :
        Binding(param, config) // Call super-class' constructor
    {

        if (!config.hasProperty("topic"))
            throw std::invalid_argument("ROS parameter binding does not specify a topic name");

        topic = config.getProperty("topic");

        PRINT_INFO_STATEMENT("InputBindingROS Created\n"
                           << " - topic: " << topic << "\n"
                           << " - address: " << this << "\n"
                           << " - parameter name: " << param->name());

        sub = nh.subscribe<DataTypeROS>(topic, 1,
            & InputBindingROS<DataTypeROS>::subscriberCallback, this);
    }

    /*!
     * The destructor.
     */
    virtual ~InputBindingROS()
    {
        PRINT_INFO_STATEMENT("Destructor called\n"
                           << " - topic: " << topic << "\n"
                           << " - address: " << this << "\n"
                           << " - parameter name: " << param->name());
        sub.shutdown();
    }

protected:

    /*!
     * The ROS topic callback method.
     */
    virtual void subscriberCallback(const boost::shared_ptr<DataTypeROS const> & msgPtr)
    {
        // Don't do anything here since specializations of this class must define this method.  See:
        // http://stackoverflow.com/questions/17118294/overloading-a-c-template-class-virtual-function
        throw std::invalid_argument("Non-functional subscriberCallback method called in base templated class!");
    }

    std::string topic;
    ros::Subscriber sub;
    DataTypeROS msg;
};

// ***************************************************************************
//  Standard message extensions
// ***************************************************************************
template<>
void InputBindingROS<std_msgs::String>::subscriberCallback(const boost::shared_ptr<std_msgs::String const> & msgPtr)
{
  msg = *msgPtr;
  param->set(msg.data);
}

template<>
void InputBindingROS<std_msgs::Int32>::subscriberCallback(const boost::shared_ptr<std_msgs::Int32 const> & msgPtr)
{
    msg = *msgPtr;
    param->set(msg.data);
}

template<>
void InputBindingROS<std_msgs::Float64>::subscriberCallback(const boost::shared_ptr<std_msgs::Float64 const> & msgPtr)
{
    msg = *msgPtr;
    param->set(msg.data);
}

template<>
void InputBindingROS<std_msgs::Float64MultiArray>::subscriberCallback(const boost::shared_ptr<std_msgs::Float64MultiArray const> & msgPtr)
{
    msg = *msgPtr;

    int rows = msg.layout.dim[0].size; // rows
    int cols = 1;

    if (msg.layout.dim.size() > 1)
    {
        cols = msg.layout.dim[1].size; // cols
    }

    if ( (cols == 1) && (param->isType(controlit::PARAMETER_TYPE_VECTOR)) )
    {
        Eigen::VectorXd v(rows);
        matrixMsgToEigen(msg, v);
        param->set(v);
    }
    else if (param->isType(controlit::PARAMETER_TYPE_MATRIX))
    {
        Eigen::MatrixXd m(rows, cols);
        matrixMsgToEigen(msg, m);
        param->set(m);
    }
};

// ***************************************************************************
//  Geometry message extensions
// ***************************************************************************

void msgcpy(controlit::Parameter * p, const geometry_msgs::Point & msg)
{
    Eigen::VectorXd pt(3);
    pt(0) = msg.x;
    pt(1) = msg.y;
    pt(2) = msg.z;
    p->set(pt);
}


template<>
void InputBindingROS<geometry_msgs::Point>::subscriberCallback(const boost::shared_ptr<geometry_msgs::Point const> & msgPtr)
{
    msg = *msgPtr;
    msgcpy(param, msg);
}

template<>
void InputBindingROS<geometry_msgs::PointStamped>::subscriberCallback(const boost::shared_ptr<geometry_msgs::PointStamped const> & msgPtr)
{
    msg = *msgPtr;
    msgcpy(param, msg.point);
}

void msgcpy(controlit::Parameter * p, const geometry_msgs::Pose & msg)
{
    if ( p->isType(controlit::PARAMETER_TYPE_VECTOR) )
    {
        // Copy into vector form
        Eigen::VectorXd pose(7);
        pose(0) = msg.position.x;
        pose(1) = msg.position.y;
        pose(2) = msg.position.z;
        pose(3) = msg.orientation.w;
        pose(4) = msg.orientation.x;
        pose(5) = msg.orientation.y;
        pose(6) = msg.orientation.z;
        p->set(pose);
    }
    else if ( p->isType(controlit::PARAMETER_TYPE_MATRIX) )
    {
        // Create homogeneous transformation matrix
        Eigen::MatrixXd pose = Eigen::MatrixXd::Identity(4, 4);
        Eigen::Quaterniond quat(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
        pose.block(0, 0, 3, 3) = quat.toRotationMatrix();

        pose(0, 3) = msg.position.x;
        pose(1, 3) = msg.position.y;
        pose(2, 3) = msg.position.z;

        p->set(pose);
    }
}

template<>
void InputBindingROS<geometry_msgs::Pose>::subscriberCallback(const boost::shared_ptr<geometry_msgs::Pose const> & msgPtr)
{
    msg = *msgPtr;
    msgcpy(param, msg);
}

template<>
void InputBindingROS<geometry_msgs::PoseStamped>::subscriberCallback(const boost::shared_ptr<geometry_msgs::PoseStamped const> & msgPtr)
{
    msg = *msgPtr;
    msgcpy(param, msg.pose);
}

void msgcpy(controlit::Parameter * p, geometry_msgs::Wrench const & msg)
{
    // Copy into vector form
    Eigen::VectorXd wrench(6);
    wrench(0) = msg.force.x;
    wrench(1) = msg.force.y;
    wrench(2) = msg.force.z;
    wrench(3) = msg.torque.x;
    wrench(4) = msg.torque.y;
    wrench(5) = msg.torque.z;
    p->set(wrench);
}

template<>
void InputBindingROS<geometry_msgs::Wrench>::subscriberCallback(const boost::shared_ptr<geometry_msgs::Wrench const> & msgPtr)
{
    msg = *msgPtr;
    msgcpy(param, msg);
}

template<>
void InputBindingROS<geometry_msgs::WrenchStamped>::subscriberCallback(const boost::shared_ptr<geometry_msgs::WrenchStamped const> & msgPtr)
{
    msg = *msgPtr;
    msgcpy(param, msg.wrench);
}

void msgcpy(controlit::Parameter * p, geometry_msgs::Twist const & msg)
{
    // Copy into vector form
    Eigen::VectorXd twist(6);
    twist(0) = msg.linear.x;
    twist(1) = msg.linear.y;
    twist(2) = msg.linear.z;
    twist(3) = msg.angular.x;
    twist(4) = msg.angular.y;
    twist(5) = msg.angular.z;
    p->set(twist);
}

template<>
void InputBindingROS<geometry_msgs::Twist>::subscriberCallback(const boost::shared_ptr<const geometry_msgs::Twist> & msgPtr)
{
    msg = *msgPtr;
    msgcpy(param, msg);
}

template<>
void InputBindingROS<geometry_msgs::TwistStamped>::subscriberCallback(const boost::shared_ptr<const geometry_msgs::TwistStamped> & msgPtr)
{
    msg = *msgPtr;
    msgcpy(param, msg.twist);

    // Copy into vector form
    // Eigen::VectorXd twist(6);
    // twist(0) = msgPtr->twist.linear.x;
    // twist(1) = msgPtr->twist.linear.y;
    // twist(2) = msgPtr->twist.linear.z;
    // twist(3) = msgPtr->twist.angular.x;
    // twist(4) = msgPtr->twist.angular.y;
    // twist(5) = msgPtr->twist.angular.z;
    // param->set(twist);

}

// ***************************************************************************
//  Sensor message extensions
// ***************************************************************************

/*!
 * Saves the whole joint state message in the provided parameter.
 */
struct JointStateBinding :
    public InputBindingROS<sensor_msgs::JointState>
{
    /*!
     * The constructor.
     *
     * \param nh The node handle.
     * \param[in] param A pointer to the parameter where the incoming sensor_msgs::JointState messages are to be saved.
     * \param[in] config The binding configuration.  This specifies details of the binding like the
     * ROS topic, data type, publish rate, queue size, etc.
     */
    JointStateBinding(ros::NodeHandle & nh, controlit::Parameter * param, controlit::BindingConfig const & config) :
        InputBindingROS<sensor_msgs::JointState>(nh, param, config)
    {
        if (not param->isType(controlit::PARAMETER_TYPE_JOINT_STATE))
            throw std::invalid_argument("Binding for sensor_msgs::JointState restricted to sensor_msgs::JointState type parameters");
    }

    /*!
     * This method is called when a sensor_msgs::JointState is received from the bound ROS topic.
     *
     * \param _msg A pointer to the received message.
     */
    virtual void subscriberCallback(const boost::shared_ptr<sensor_msgs::JointState const>& msgPtr)
    {
      msg = *msgPtr;
      param->set(msg);
    }
};

// ***************************************************************************
//  Visualization message extensions
// ***************************************************************************
template<>
void InputBindingROS<visualization_msgs::Marker>::subscriberCallback(const boost::shared_ptr<visualization_msgs::Marker const> & msgPtr)
{
    msg = *msgPtr;

    if (config.hasProperty("bind_only_position"))
    {
        Eigen::VectorXd pos(3);
        pos(0) = msg.pose.position.x;
        pos(1) = msg.pose.position.y;
        pos(2) = msg.pose.position.z;
        param->set(pos);
        return;
    }

    if (config.hasProperty("bind_only_orientation"))
    {
        Eigen::VectorXd q(4);
        q(0) = msg.pose.orientation.w;
        q(1) = msg.pose.orientation.x;
        q(2) = msg.pose.orientation.y;
        q(3) = msg.pose.orientation.z;
        param->set(q);
        return;
    }

    msgcpy(param, msg.pose);
}

} // namespace binding_factory_library
} // namespace controlit

#endif // __CONTROLIT_BINDING_FACTORY_LIBRARY_INPUT_BINDING_ROS_HPP__
