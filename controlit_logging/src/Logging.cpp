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

#include <fstream>
#include <memory>
#include <mutex>
#include <thread>
#include <cstdlib>

#include <controlit/logging/Logging.hpp>
#include <controlit/addons/ros/ROSParameterAccessor.hpp>

// using Json::Value;
// using Json::Reader;

namespace controlit {
namespace logging {

__thread bool _init_called = false;

// Whether the logging configuration was loaded
bool configsLoaded = false;

// By default, use log priority level WARNING
Priority _log_level = Priority::Warning;

// By default, no fields are printed to the console
std::vector<std::string> _fields_to_print;

std::mutex _init_mux;

void _init(std::string const& srcdir, std::string const& pkgname)
{
    std::lock_guard<std::mutex> lock(_init_mux);
    _init_called = true;

    // check the ROS parameter server for the log configuration
    if(not configsLoaded)
    {
        // Create a ROSParameterAccessor
        ros::NodeHandle nh;
        controlit::addons::ros::ROSParameterAccessor paramAccessor(nh);

        std::stringstream ss;

        // Load the log level
        std::string logLevelString;
        if (nh.getParam("controlit/log_level", logLevelString))
        {
            ss << "Debug level: " << logLevelString << "\n";

            if(logLevelString == "DEBUG")
                _log_level = Priority::Debug;
            else if(logLevelString == "INFO")
                _log_level = Priority::Info;
            else if (logLevelString == "WARNING" || logLevelString == "WARN")
                _log_level = Priority::Warning;
            else if(logLevelString == "ERROR")
                _log_level = Priority::Error;
            else if(logLevelString == "FATAL")
                _log_level = Priority::Fatal;
            else
            {
                std::cout << "Warning, invalid log level " << logLevelString << " for package " << pkgname << "! Resorting to log level DEBUG." << std::endl;
                _log_level = Priority::Debug;
            }
        }
        else
        {
            std::cout << "Warning, no log level set for package " << pkgname << "! Resorting to log level DEBUG." << std::endl;
            _log_level = Priority::Debug;
        }

        ss << "Fields:\n";

        // Load the log fields
        std::vector<std::string> * fields = nullptr;
        if (paramAccessor.loadParameter("controlit/log_fields", &fields))
        {
            for(std::vector<std::string>::iterator it = fields->begin(); it != fields->end(); ++it)
            {
                std::string currField = *it;
                _fields_to_print.push_back(currField);
                ss << " - " << currField << "\n";
            }
        }
        else
        {
            std::cout << "controlit::logger::Logging: Warning, no fields specified." << std::endl;
        }
        
        configsLoaded = true;
    }
}

} // namespace common
} // namespace controlit
