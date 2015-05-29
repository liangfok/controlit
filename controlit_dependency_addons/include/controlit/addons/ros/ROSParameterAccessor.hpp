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

#ifndef __CONTROLIT_ADDONS_ROS_PARAMETER_ACCESSOR_HPP__
#define __CONTROLIT_ADDONS_ROS_PARAMETER_ACCESSOR_HPP__

#include <ros/ros.h>

#include <controlit/addons/eigen/LinearAlgebra.hpp>

namespace controlit {
namespace addons {
namespace ros {

using controlit::addons::eigen::Vector;

/**
 * Provides methods for accessing values stored on the ROS parameter server.
 */
class ROSParameterAccessor
{
public:
    /*!
     * The constructor.
     *
     * @param[in] nh The ROS node handle from which the parameters are to be accessed.
     */
    ROSParameterAccessor(::ros::NodeHandle & nh);

    /*!
     * The destructor.
     */
    ~ROSParameterAccessor();

    /*!
     * Returns the namespace under which all parameters are accessed.
     *
     * \return The namespace.
     */
    std::string getNamespace();

    /*!
     * Loads a boolean value from the ROS parameter server.
     *
     * \param[in] paramName The name of the ROS parameter from which to read.
     * \param[out] dest Where the resulting value should be stored.
     * \return Whether the read was successful.
     */
    // bool loadParameter(const std::string paramName, bool & dest);

    /*!
     * Loads a string value from the ROS parameter server.
     *
     * \param[in] paramName The name of the ROS parameter from which to read.
     * \param[out] dest Where the resulting value should be stored.
     * \return Whether the read was successful.
     */
    // bool loadParameter(const std::string paramName, std::string & dest);

    /*!
     * Loads an array of double values from the ROS parameter server.
     *
     * \param[in] paramName The name of the ROS parameter from which to read.
     * \param[out] dest Where the resulting value should be stored.
     * \return Whether the read was successful.
     */
    bool loadParameter(const std::string paramName, Vector & dest);

    /*!
     * Loads an double value from the ROS parameter server.
     *
     * \param[in] paramName The name of the ROS parameter from which to read.
     * \param[out] dest Where the resulting value should be stored.
     * \return Whether the read was successful.
     */
    // bool loadParameter(const std::string paramName, double & dest);

    /*!
     * Loads an uint32_t value from the ROS parameter server.
     *
     * \param[in] paramName The name of the ROS parameter from which to read.
     * \param[out] dest Where the resulting value should be stored.
     * \return Whether the read was successful.
     */
    // bool loadParameter(const std::string paramName, uint32_t & dest);

    /*!
     * Loads a vector of string values from the ROS parameter server.
     *
     * \param[in] paramName The name of the ROS parameter from which to read.
     * \param[out] dest Where the resulting value should be stored.
     * \return Whether the read was successful.
     */
    bool loadParameter(const std::string paramName, std::vector<std::string> ** dest);

    /*!
     * Loads a vector of string vectors from the ROS parameter server.
     *
     * \param[in] paramName The name of the ROS parameter from which to read.
     * \param[out] dest Where the resulting value should be stored.
     * \return Whether the read was successful.
     */
    bool loadParameter(const std::string paramName, std::vector<std::vector<std::string>> ** dest);

private:

    /*!
     * Check whether a parameter exists. This method will check up to 10 times before
     * aborting.
     * 
     * \param[in] paramName The parameter to look for.
     * \return Whether the parameter exists.
     */
    bool paramExists(const std::string paramName);

    /*!
     * The node handle used by this object.
     */
    std::unique_ptr< ::ros::NodeHandle> nh;
};

} // namespace ros
} // namespace addons
} // namespace controlit

#endif // __CONTROLIT_ADDONS_ROS_PARAMETER_ACCESSOR_HPP__
