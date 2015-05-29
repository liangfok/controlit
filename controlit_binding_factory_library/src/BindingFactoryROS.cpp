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

#include <controlit/binding_factory_library/BindingFactoryROS.hpp>

#include <controlit/logging/RealTimeLogging.hpp>

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

#include <controlit/binding_factory_library/InputBindingROS.hpp>
#include <controlit/binding_factory_library/OutputBindingROS.hpp>


namespace controlit {
namespace binding_factory_library {

BindingFactoryROS::BindingFactoryROS() :
    BindingFactory() // Call super-class' constructor
{
	// PRINT_DEBUG_STATEMENT("BindingFactoryROS Created");
}

BindingFactoryROS::~BindingFactoryROS()
{
}

Binding * BindingFactoryROS::createBinding(ros::NodeHandle & nh, const BindingConfig & config,
        Parameter * param)
{
    // CONTROLIT_INFO << "Method called!\n"
    //                << "  - " << config.toString("     ") << "\n"
    //                << "  - parameter type = " << controlit::Parameter::parameterTypeToString(param->type());

    if (config.getTransportType() == "ROSTopic")
    {
        if (config.getDirection() == controlit::BindingConfig::Input)
            return createInputBinding(nh, config, param);
        else
            return createOutputBinding(nh, config, param);
    }

    return nullptr;
 }

Binding * BindingFactoryROS::createInputBinding(ros::NodeHandle & nh,
    const controlit::BindingConfig & config, controlit::Parameter * param)
{
    if(config.getTransportDataType() == "std_msgs/String" && param->type() == controlit::PARAMETER_TYPE_STRING)
        return new InputBindingROS<std_msgs::String>(nh, param, config);

    else if(config.getTransportDataType() == "std_msgs/Int32" && param->type() == controlit::PARAMETER_TYPE_INTEGER)
        return new InputBindingROS<std_msgs::Int32>(nh, param, config);

    else if(config.getTransportDataType() == "std_msgs/Float64" && param->type() == controlit::PARAMETER_TYPE_REAL)
        return new InputBindingROS<std_msgs::Float64>(nh, param, config);

    else if(config.getTransportDataType() == "std_msgs/Float64MultiArray" && (param->type() == controlit::PARAMETER_TYPE_VECTOR || param->type() == controlit::PARAMETER_TYPE_MATRIX))
        return new InputBindingROS<std_msgs::Float64MultiArray>(nh, param, config);

    else if(config.getTransportDataType() == "geometry_msgs/Point" && param->type() == controlit::PARAMETER_TYPE_VECTOR)
        return new InputBindingROS<geometry_msgs::Point>(nh, param, config);

    else if(config.getTransportDataType() == "geometry_msgs/PointStamped" && param->type() == controlit::PARAMETER_TYPE_VECTOR)
        return new InputBindingROS<geometry_msgs::PointStamped>(nh, param, config);

    else if(config.getTransportDataType() == "geometry_msgs/Pose" && (param->type() == controlit::PARAMETER_TYPE_VECTOR || param->type() == controlit::PARAMETER_TYPE_MATRIX))
        return new InputBindingROS<geometry_msgs::Pose>(nh, param, config);

    else if(config.getTransportDataType() == "geometry_msgs/PoseStamped" && (param->type() == controlit::PARAMETER_TYPE_VECTOR || param->type() == controlit::PARAMETER_TYPE_MATRIX))
        return new InputBindingROS<geometry_msgs::PoseStamped>(nh, param, config);

    else if(config.getTransportDataType() == "geometry_msgs/Wrench" && param->type() == controlit::PARAMETER_TYPE_VECTOR)
        return new InputBindingROS<geometry_msgs::Wrench>(nh, param, config);

    else if(config.getTransportDataType() == "geometry_msgs/WrenchStamped" && param->type() == controlit::PARAMETER_TYPE_VECTOR)
        return new InputBindingROS<geometry_msgs::WrenchStamped>(nh, param, config);

    else if(config.getTransportDataType() == "geometry_msgs/Twist"  && param->type() == controlit::PARAMETER_TYPE_VECTOR)
        return new InputBindingROS<geometry_msgs::Twist>(nh, param, config);

    else if(config.getTransportDataType() == "geometry_msgs/TwistStamped" && param->type() == controlit::PARAMETER_TYPE_VECTOR)
        return new InputBindingROS<geometry_msgs::TwistStamped>(nh, param, config);

    // else if(config.getTransportDataType() == "sensor_msgs/JointState" && param->type() == controlit::PARAMETER_TYPE_VECTOR)
    //     return new InputBindingROSJointStateToEigenVector(nh, param, config);

    else if(config.getTransportDataType() == "sensor_msgs/JointState" && param->type() == controlit::PARAMETER_TYPE_JOINT_STATE)
        return new JointStateBinding(nh, param, config);

    else if(config.getTransportDataType() == "visualization_msgs/Marker"  && param->type() == controlit::PARAMETER_TYPE_VECTOR)
        return new InputBindingROS<visualization_msgs::Marker>(nh, param, config);
    else
    {
        CONTROLIT_WARN << "Unable to instantiate an appropriate InputBindingROS object!\n"
            << "  - " << config.toString("     ") << "\n"
            << "  - parameter type = " << controlit::Parameter::parameterTypeToString(param->type());
        return nullptr;
    }
}

Binding * BindingFactoryROS::createOutputBinding(ros::NodeHandle & nh,
    const controlit::BindingConfig & config, controlit::Parameter * param)
{
    // CONTROLIT_INFO << "Method called!\n"
    //                << "  - config.getTransportDataType(): \"" << config.getTransportDataType() << "\"" << "\n"
    //                << "  - param->type(): " << controlit::Parameter::parameterTypeToString(param->type());

    if(config.getTransportDataType() == "std_msgs/String" && param->type() == controlit::PARAMETER_TYPE_STRING)
        return new OutputBindingROS<std_msgs::String, std::string>(nh, param, config);

    else if(config.getTransportDataType() == "std_msgs/Int32" && param->type() == controlit::PARAMETER_TYPE_INTEGER)
        return new OutputBindingROS<std_msgs::Int32, int>(nh, param, config);

    else if(config.getTransportDataType() == "std_msgs/Float64" && param->type() == controlit::PARAMETER_TYPE_REAL)
        return new OutputBindingROS<std_msgs::Float64, double>(nh, param, config);

    else if(config.getTransportDataType() == "std_msgs/Float64MultiArray" && param->type() == controlit::PARAMETER_TYPE_VECTOR)
        return new OutputBindingROS<std_msgs::Float64MultiArray, Vector>(nh, param, config);

    else if(config.getTransportDataType() == "std_msgs/Float64MultiArray" && param->type() == controlit::PARAMETER_TYPE_MATRIX)
        return new OutputBindingROS<std_msgs::Float64MultiArray, Matrix>(nh, param, config);

    else if(config.getTransportDataType() == "geometry_msgs/Point" && param->type() == controlit::PARAMETER_TYPE_VECTOR)
        return new OutputBindingROS<geometry_msgs::Point, Vector>(nh, param, config);

    else if(config.getTransportDataType() == "geometry_msgs/PointStamped" && param->type() == controlit::PARAMETER_TYPE_VECTOR)
        return new OutputBindingROS<geometry_msgs::PointStamped, Vector>(nh, param, config);

    else if(config.getTransportDataType() == "geometry_msgs/Pose" && param->type() == controlit::PARAMETER_TYPE_VECTOR)
        return new OutputBindingROS<geometry_msgs::Pose, Vector>(nh, param, config);

    else if(config.getTransportDataType() == "geometry_msgs/PoseStamped" && param->type() == controlit::PARAMETER_TYPE_VECTOR)
        return new OutputBindingROS<geometry_msgs::PoseStamped, Vector>(nh, param, config);

    else if(config.getTransportDataType() == "geometry_msgs/Pose" && param->type() == controlit::PARAMETER_TYPE_MATRIX)
        return new OutputBindingROS<geometry_msgs::Pose, Matrix>(nh, param, config);

    else if(config.getTransportDataType() == "geometry_msgs/PoseStamped" && param->type() == controlit::PARAMETER_TYPE_MATRIX)
        return new OutputBindingROS<geometry_msgs::PoseStamped, Matrix>(nh, param, config);

    else if(config.getTransportDataType() == "geometry_msgs/Wrench" && param->type() == controlit::PARAMETER_TYPE_VECTOR)
        return new OutputBindingROS<geometry_msgs::Wrench, Vector>(nh, param, config);

    else if(config.getTransportDataType() == "geometry_msgs/WrenchStamped" && param->type() == controlit::PARAMETER_TYPE_VECTOR )
        return new OutputBindingROS<geometry_msgs::WrenchStamped, Vector>(nh, param, config);

    else if(config.getTransportDataType() == "geometry_msgs/Twist" && param->type() == controlit::PARAMETER_TYPE_VECTOR)
        return new OutputBindingROS<geometry_msgs::Twist, Vector>(nh, param, config);

    else if(config.getTransportDataType() == "geometry_msgs/TwistStamped" && param->type() == controlit::PARAMETER_TYPE_VECTOR)
        return new OutputBindingROS<geometry_msgs::TwistStamped, Vector>(nh, param, config);

    else if(config.getTransportDataType() == "visualization_msgs/Marker" && param->type() == controlit::PARAMETER_TYPE_VECTOR)
        return new OutputBindingROS<visualization_msgs::Marker, Vector>(nh, param, config);
    else
    {
        CONTROLIT_WARN << "Unable to instantiate an appropriate OutputBindingROS object!\n"
            << "  - " << config.toString("     ") << "\n"
            << "  - parameter type = " << controlit::Parameter::parameterTypeToString(param->type());
        return nullptr;
    }
}

} // namespace binding_factory_library
} // namespace controlit
