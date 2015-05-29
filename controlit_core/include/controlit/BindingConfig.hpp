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

#ifndef __CONTROLIT_CORE_BINDING_CONFIG_HPP__
#define __CONTROLIT_CORE_BINDING_CONFIG_HPP__

#include <string>
#include <sstream>
#include <list>
#include <map>

#include <controlit/UniqueObject.hpp>

namespace controlit {

class BindingConfig : public UniqueObject
{
public:
    typedef std::map<std::string, std::string> Property_t;

    enum Direction
    {
        Undefined = 0,
        Input,
        Output,
        Bidirectional
    };

    /*!
     * A constructor.
     */
    BindingConfig();

    /*!
     * A constructor.
     *
     * \param[in] transportType
     * \param[in] transportDataType
     * \param[in] topic
     * \param[in] direction
     */
    BindingConfig(std::string const & transportType, std::string const & transportDataType,
        std::string const & topic, Direction direction);

    /*!
     * Sets the transport type.
     */
    void setTransportType(std::string const& transportType);

    /*!
     * Sets the transport data type.
     */
    void setTransportDataType(std::string const& transportDataType);

    /*!
     * Sets the direction.
     */
    void setDirection(Direction direction);

    /*!
     *
     */
    std::string name() const;

    /*!
     *
     */
    bool operator==(BindingConfig const& rhs);

    /*!
     *
     */
    void setParameter(std::string const& paramName);

    /*!
     * Checks whether this BindingConfig includes the specified parameter.
     *
     * \return true if the parameter is included in this binding configuration.
     */
    // bool containsParameter(std::string const& paramName) const;

    /*!
     * Checks whether this BindingConfig is for the specified parameter.
     *
     * \return true if the parameter is included in this binding configuration.
     */
    bool isForParameter(std::string const& paramName) const;

    /*!
     *
     */
    bool hasProperty(std::string const& key) const;

    /*!
     *
     */
    void addProperty(std::string const& key, std::string const& value);

    /*!
     *
     */
    std::string getProperty(std::string const& key) const;

    /*!
     * \return all of the properties in this BindingConfig.
     */
    const Property_t & getProperties() const { return properties; }

    /*!
     * \return The bound parameter.
     */
    const std::string & getParameter() const { return parameter; }

    /*!
     *
     */
    template <class T>
    T getTypedProperty(std::string const& key) const
    {
        std::stringstream sstr(getProperty(key));

        T val;
        sstr >> val;

        return val;
    }

    template <class T>
    std::vector<T> getTypedVectorProperty(std::string const& key) const
    {
        std::stringstream sstr(getProperty(key));
        std::vector<T> values;

        T val;
        while (sstr >> val) values.push_back(val);

        return values;
    }

    /*!
     * \return the binding direction.
     */
    const Direction & getDirection() const
    {
        return direction;
    }

    /*!
     * Returns the name of the transport layer.  For example, this can be
     * "ROSTopic" if ROS topics are used.
     *
     * \return the transport type.
     */
    const std::string & getTransportType() const
    {
        return transportType;
    }

    /*!
     * Returns a string that describes the transport layer's data type.
     * For example, if ROS topics are used, this would be the name of the
     * ROS message type.
     *
     * \return the transport data type.
     */
    const std::string & getTransportDataType() const
    {
        return transportDataType;
    }

    /*!
     * Takes as input a binding direction and returns a string representation
     * of it.  This is useful for generating meaningful debug messages, e.g.,
     * instead of 1 the message will have "Input".
     *
     * \param direction The binding direction.
     * \return A string representation of the binding direction.
     */
    static std::string bindingDirectionToString(Direction direction);

    /*!
     * Returns a string representation of this class.
     *
     * \return A string representation of this class.
     */
    std::string toString() const;

    /*!
     * Returns a string representation of this class.
     *
     * \param[in] prefix A string to add to the beginning of each line.
     * \return A string representation of this class.
     */
    std::string toString(std::string prefix) const;

private:

    /*!
     * The binding direction.
     */
    Direction direction;

    /*!
     * A string specifying the transport layer type.  For example, this can
     * be "ROS", "SM", etc.
     */
    std::string transportType;

    /*!
     * The data type used in the transport layer.  For example, this can be
     * "std_msgs/String", "geometry_msgs/PoseStamped", etc.
     */
    std::string transportDataType;

    /*!
     * The parameter to be bound.
     */
    std::string parameter;

    /*!
     * A list of properties.
     */
    Property_t properties;
};

} // namespace controlit

#endif // __CONTROLIT_CORE_BINDING_CONFIG_HPP__
