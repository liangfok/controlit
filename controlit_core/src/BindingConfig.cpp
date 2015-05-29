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

#include <sstream>
#include <boost/uuid/uuid_io.hpp>

#include <controlit/BindingConfig.hpp>

namespace controlit {

BindingConfig::BindingConfig() :
    UniqueObject(),
    direction(BindingConfig::Direction::Undefined),
    transportType("__TRANSPORT_TYPE_NOT_DEFINED__"),
    transportDataType("__TRANSPORT_DATA_TYPE_NOT_DEFINED__")
{
}

BindingConfig::BindingConfig(std::string const& transportType, std::string const& transportDataType,
    std::string const& topic, BindingConfig::Direction direction) :
    UniqueObject(),
    direction(direction),
    transportType(transportType),
    transportDataType(transportDataType)
{
    addProperty("topic", topic);
}

void BindingConfig::setTransportType(std::string const& transportType)
{
    this->transportType = transportType;
}

void BindingConfig::setTransportDataType(std::string const& transportDataType)
{
    this->transportDataType = transportDataType;
}

void BindingConfig::setDirection(Direction direction)
{
    this->direction = direction;
}

std::string BindingConfig::name() const
{
    std::ostringstream ostr;
    ostr << "binding_" << id();
    return ostr.str();
}

bool BindingConfig::operator==(BindingConfig const& rhs)
{
    // TODO: Check that properties match
    return (this->transportType == rhs.getTransportType()) &&
           (this->transportDataType == rhs.getTransportDataType()) &&
           (this->parameter.compare(rhs.parameter) == 0) &&
           (this->direction == rhs.direction);
}

void BindingConfig::setParameter(std::string const& parameter)
{
    this->parameter = parameter;
}

bool BindingConfig::isForParameter(std::string const& parameter) const
{
    return this->parameter.compare(parameter) == 0;

    // for (std::list<std::string>::const_iterator paramIter = parameters.begin(); paramIter != parameters.end(); paramIter++)
    // {
    //     if (parameter.compare(*paramIter) == 0)
    //         return true;
    // }
    // return false;
}

bool BindingConfig::hasProperty(std::string const& key) const
{
    auto const& tuple = properties.find(key);
    return (tuple != properties.end());
}

void BindingConfig::addProperty(std::string const& key, std::string const& value)
{
    auto tuple = properties.find(key);
    if (tuple == properties.end())
    {
        properties[key] = value;
    }
}

std::string BindingConfig::getProperty(std::string const& key) const
{
    auto const& tuple = properties.find(key);
    if (tuple != properties.end())
    {
        return tuple->second;
    }

    throw std::invalid_argument(key + " not a valid property.");
}

std::string BindingConfig::bindingDirectionToString(Direction direction)
{
    switch(direction)
    {
        case Undefined:
            return "Undefined";
        case Input:
            return "Input";
        case Output:
            return "Output";
        case Bidirectional:
            return "Bidirectional";
        default:
            return "Unknown";
    }
}


std::string BindingConfig::toString() const
{
    std::string prefix("");
    return toString(prefix);
}

std::string BindingConfig::toString(std::string prefix) const
{
    // Create a string containing the parameters
    // std::stringstream paramString;
    // for (std::list<std::string>::const_iterator paramIter = parameters.begin(); paramIter != parameters.end(); paramIter++)
    // {
    //     paramString << *paramIter;
    //     paramString << ", ";
    // }

    // Create a string containing details of this Bindingconfig
    std::stringstream buff;
    buff
        << "BindingConfig:\n"
        << prefix << " - name: " << name() << "\n"
        << prefix << " - transportType: " << transportType << "\n"
        << prefix << " - transportDataType: " << transportDataType << "\n"
        << prefix << " - direction: " << bindingDirectionToString(direction) << "\n"
        << prefix << " - parameter: " << parameter << "\n" //<< paramString.str().substr(0, paramString.str().size() - 2) << "]\n"
        << prefix << " - properties:\n";

    for (Property_t::const_iterator iter = properties.begin(); iter != properties.end(); ++iter)
    {
        buff << prefix << "    - " << iter->first << ": " << iter->second << "\n";
    }

    return buff.str().substr(0, buff.str().size() - 1);  // subtract 1 to remove trailing new line character
}

} // namespace controlit
