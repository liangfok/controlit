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

#include <controlit/exec/ControlItExec.hpp>
#include <controlit/addons/ros/ROSParameterAccessor.hpp>
#include <controlit/Coordinator.hpp>

namespace controlit {
namespace exec {

ControlItExec::ControlItExec() :
  initialized(false)
  // coordinator(nullptr)
{
}

ControlItExec::~ControlItExec()
{
}

bool ControlItExec::init()
{
    // coordinator.reset(new controlit::Coordinator());
    if (coordinator.init())
    {
        initialized = true;
        return true;
    }
    else
        return false;
}

bool ControlItExec::start()
{
    return coordinator.start();
}

bool ControlItExec::stop()
{
    return coordinator.stop();
}


std::string ControlItExec::toString(std::string const& prefix) const
{
    ros::NodeHandle nh;

    std::stringstream ss;
    ss << prefix << "ControlItExec details:\n";
    ss << prefix << "  - initialized: " << (initialized ? "true" : "false") << "\n";
    ss << prefix << "  - controller name: " << nh.getNamespace();

    return ss.str();
}

} // namespace exec
} // namespace controlit


// This is the main method that starts everything.
int main(int argc, char **argv)
{
    // Define usage
    std::stringstream ss;
    ss << "Usage: rosrun controlit_exec controlit_exec [options]\n"
       << "Valid options include:\n"
       << "  -h: display this usage string\n"
       << "Note: The controller's name is specified by the ROS nodehandle's namespace.";

    ros::init(argc, argv, "ControlItExec");

    if (argc != 1)
    {
        // Parse the command line arguments
        int option_char;
        while ((option_char = getopt(argc, argv, "h")) != -1)
        {
            switch (option_char)
            {
                case 'h':
                    std::cout << ss.str() << std::endl;
                    return 0;
                    break;
                default:
                    std::cerr << "ControlItExec: ERROR: Unknown option " << option_char << ".  " << ss.str() << std::endl;
                    return -1;
            }
        }
    }

    ros::NodeHandle nh;

    std::cout << "ControlItExec: Starting controller \"" << nh.getNamespace() << "\"..." << std::endl;

    // Create and start a ControlItExec
    controlit::exec::ControlItExec controlitExec;
    if (controlitExec.init())
    {
        if (controlitExec.start())
        {
            // Loop until someone hits ctrl+c
            ros::Rate loop_rate(1000);

            while (ros::ok())
            {
                ros::spinOnce();
                loop_rate.sleep();
            }

            // Stop ControlItExec
            controlitExec.stop();
        }
        else
        {
            std::cerr << "ControlItExec: ERROR: Failed to start coordinator." << std::endl;
        }
    }
    else
    {
        std::cerr << "ControlItExec: ERROR: Failed to initialize coordinator." << std::endl;
    }
}