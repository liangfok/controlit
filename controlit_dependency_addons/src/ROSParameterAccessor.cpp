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

#include <controlit/addons/ros/ROSParameterAccessor.hpp>

namespace controlit {
namespace addons {
namespace ros {

ROSParameterAccessor::ROSParameterAccessor(::ros::NodeHandle & nh)
{
    // Create a new node handle with the same namespace as the supplied node handle.
    // This ensures we always have a valid node handle when we load the ControlIt!
    // parameters.
    this->nh.reset(new ::ros::NodeHandle(nh.getNamespace()));
}

ROSParameterAccessor::~ROSParameterAccessor()
{
}

std::string ROSParameterAccessor::getNamespace()
{
    return nh->getNamespace();
}

bool ROSParameterAccessor::paramExists(const std::string paramName)
{
    int count = 0;

    bool hasParam = false;

    while (!(hasParam = nh->hasParam(paramName)) && count++ < 20)
    {
    }

    return hasParam;
}

bool ROSParameterAccessor::loadParameter(const std::string paramName, Vector & dest)
{
    if (!paramExists(paramName)) return false;

    XmlRpc::XmlRpcValue xml_array;

    // Get the ROS parameter
    if (!nh->getParam(paramName, xml_array))
    {
        // ROS_WARN_STREAM("ROS parameter '" << nh->getNamespace() << "/" << paramName << "' does not exist.");
        return false;
    }

    // Ensure the parameter is of type array
    if(xml_array.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        std::stringstream errorMsg;
        errorMsg << "ROS parameter '" << nh->getNamespace() << "/" << paramName
                 << "' contained a non-array type (type = " << xml_array.getType() << ").";

        ROS_ERROR_STREAM(errorMsg.str());
        return false;
    }

    // Save the array of doubles
    dest.setZero(xml_array.size());

    for (int ii = 0; ii < dest.size(); ++ii)
    {
        if(xml_array[ii].getType() == XmlRpc::XmlRpcValue::TypeDouble)
        {
            dest(ii) = static_cast<double>(xml_array[ii]);
        }
        else if(xml_array[ii].getType() == XmlRpc::XmlRpcValue::TypeInt)
        {
            dest(ii) = static_cast<int>(xml_array[ii]);
        }
        else
        {
            std::stringstream errorMsg;
            errorMsg << __func__ << ": Problems when loading parameter '" << paramName << "': "
                "xml_array[" << ii << "] is not of type double or int, type = " << xml_array[ii].getType();

            ROS_ERROR_STREAM(errorMsg.str());
            return false;
        }
    }

    return true;
}

bool ROSParameterAccessor::loadParameter(const std::string paramName, std::vector<std::string> ** dest)
{
    if (!paramExists(paramName)) return false;

    XmlRpc::XmlRpcValue xml_array;

    // Get the ROS parameter
    if (!nh->getParam(paramName, xml_array))
    {
        return false;
    }

    // Ensure the parameter is of type array
    if(xml_array.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        std::stringstream errorMsg;
        errorMsg << "ROS parameter '" << nh->getNamespace() << "/" << paramName
            << "' contained a non-array type (type = " << xml_array.getType() << ").";

        ROS_ERROR_STREAM(errorMsg.str());
        return false;
    }

    // Save the vector of strings
    if (*dest != nullptr)
        delete *dest;

    *dest = new std::vector<std::string>(xml_array.size());

    for (int ii = 0; ii < xml_array.size(); ++ii)
    {
        if(xml_array[ii].getType() == XmlRpc::XmlRpcValue::TypeString)
        {
            (*(*dest))[ii] = static_cast<std::string>(xml_array[ii]);
        }
        else
        {
          std::stringstream errorMsg;
          errorMsg << __func__ << ": Problems when loading parameter '" << paramName << "': "
              "xml_array[" << ii << "] is not of type string, type = " << xml_array[ii].getType();

          ROS_ERROR_STREAM(errorMsg.str());
          return false;
        }
    }

    return true;
}


bool ROSParameterAccessor::loadParameter(const std::string paramName, std::vector<std::vector<std::string>> ** dest)
{
    if (!paramExists(paramName)) return false;

    XmlRpc::XmlRpcValue xml_array;

    // Get the ROS parameter
    if (!nh->getParam(paramName, xml_array))
    {
        return false;
    }

    // Ensure the parameter is of type array
    if(xml_array.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        std::stringstream errorMsg;
        errorMsg << "ROS parameter '" << nh->getNamespace() << "/" << paramName
           << "' contained a non-array type (type = " << xml_array.getType() << ").";

        ROS_ERROR_STREAM(errorMsg.str());

        return false;
    }

    // Save the vector of strings
    if (*dest != NULL)
        delete *dest;
    *dest = new std::vector<std::vector<std::string>>(xml_array.size());


    for (int ii = 0; ii < xml_array.size(); ++ii)
    {
        if(xml_array[ii].getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            std::vector<std::string> stringVec(xml_array[ii].size());

              for (int jj = 0; jj < xml_array[ii].size(); jj++)
              {
                  if(xml_array[ii][jj].getType() == XmlRpc::XmlRpcValue::TypeString)
                  {
                      stringVec[jj] = static_cast<std::string>(xml_array[ii][jj]);
                  }
                  else
                  {
                      std::stringstream errorMsg;
                      errorMsg << __func__ << ": Problems when loading parameter '" << paramName << "': "
                        "xml_array[" << ii << "] is not of type string, type = " << xml_array[ii].getType();

                      ROS_ERROR_STREAM(errorMsg.str());
                      return false;
                  }
            }

            (*(*dest))[ii]=stringVec;
        }
        else
        {
            std::stringstream errorMsg;
            errorMsg << __func__ << ": Problems when loading parameter '" << paramName << "': "
                "xml_array[" << ii << "] is not of type array, type = " << xml_array[ii].getType();

            ROS_ERROR_STREAM(errorMsg.str());
            return false;
        }
    }

    return true;
}

} // namespace ros
} // namespace addons
} // namespace controlit
