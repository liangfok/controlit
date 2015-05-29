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

#include <controlit/robot_interface_library/SMWatchdog.hpp>

void destroySharedMemory(int param)
{
 std::cerr << "SMWatchdog: Destroying shared memory space." << std::endl;
 std::string interface_name = "smci";
 std::string mutex_name = interface_name + "_mutex";
 std::string data_name = interface_name + "_data";
 boost::interprocess::shared_memory_object::remove(data_name.c_str());
 boost::interprocess::named_mutex::remove(mutex_name.c_str());
  // SharedMemoryControlInterface::destroyMemory("smci");
 std::cerr << "SMWatchdog: Done destroying shared memory space." << std::endl;
}

namespace controlit
{
  namespace robot_interface_library
  {
    SMWatchdog::SMWatchdog(const ros::NodeHandle& nh) :
        m_nh(nh)
    {
      m_nh.param("loop_rate", m_loop_rate, 10.0);
    }

    SMWatchdog::~SMWatchdog()
    {
      destroySharedMemory(0);
    }

    void SMWatchdog::spin()
    {
      ROS_INFO("SMWatchdog started.");
      ros::Rate loop_rate(m_loop_rate);
      while(ros::ok())
      {
        ros::spinOnce();
        loop_rate.sleep();
      }
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "topic_node_template");
  ros::NodeHandle nh("~");

  signal(SIGABRT, destroySharedMemory);
  signal(SIGFPE , destroySharedMemory);
  signal(SIGILL, destroySharedMemory);
  signal(SIGINT, destroySharedMemory);
  signal(SIGSEGV, destroySharedMemory);
  signal(SIGTERM, destroySharedMemory);

  controlit::robot_interface_library::SMWatchdog node(nh);
  node.spin();

  return 0;
}
