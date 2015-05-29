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

#ifndef _CONTROLIT_STRING_UTILITIES_HPP_
#define _CONTROLIT_STRING_UTILITIES_HPP_

// #include <drc/common/string_utilities.hpp>
// #include <controlit/addons/eigen/LinearAlgebra.hpp>
#include <sensor_msgs/JointState.h>

namespace controlit {
namespace utility {

// using drc::common::string_splitter;

/*!
 * Generates a string that consists of:
 *
 * [prefix] [index] [joint name]: [command],
 * [prefix] [index] [joint name]: [command], ...
 *
 * This is useful for printing the commands issued by the tasks
 * and the whole body controller.
 *
 * \param jointNames The names of the joints.
 * \param commands The commands issued to each of the joints.
 * \param prefix A string to insert at the beginning of each line.
 */
inline std::string prettyPrintJointSpaceCommand(const std::vector<std::string> & jointNames,
  const Vector & commands, const std::string & prefix)
{
  std::stringstream commandString;

  for (size_t ii = 0; ii < jointNames.size(); ii++)
  {
    commandString << prefix << "(" << ii << ") " << jointNames[ii] << ": " << commands[ii];
    if (ii < jointNames.size() - 1)
      commandString << ",\n";
  }

  return commandString.str();
}

/*!
 * Returns a string representation of a sensor_msgs::JointState message.
 *
 * \param[in] msg The JointStateMsg to print.
 * \param[in] prefix A string to add to the beginning of each line in the resulting string.
 * \return A string representation of the JointStateMsg.
 */
inline std::string prettyPrintJointStateMsg(const sensor_msgs::JointState & msg,
  const std::string prefix = "")
{
  std::stringstream buff;

  buff << prefix << "Header:\n";
  buff << prefix << "seq: " << msg.header.seq << "\n";
  buff << prefix << "stamp: " << ros::Time(msg.header.stamp.sec, msg.header.stamp.nsec).toSec() << "\n";
  buff << prefix << "frame_id: " << msg.header.frame_id << "\n";

  size_t numDOFs = msg.name.size();

  buff << prefix << "    Name: [";

  for (size_t ii = 0; ii < numDOFs; ii++)
  {
    buff << msg.name[ii];
    if (ii < numDOFs - 1)
      buff << ", ";
  }

  buff << "]\n";

  numDOFs = msg.position.size();

  buff << prefix << "    Position: [";

  for (size_t ii = 0; ii < numDOFs; ii++)
  {
    buff << msg.position[ii];
    if (ii < numDOFs - 1)
      buff << ", ";
  }

  buff << "]\n";

  numDOFs = msg.velocity.size();

  buff << prefix << "    Velocity: [";

  for (size_t ii = 0; ii < numDOFs; ii++)
  {
    buff << msg.velocity[ii];
    if (ii < numDOFs - 1)
      buff << ", ";
  }

  buff << "]\n";


  numDOFs = msg.effort.size();

  buff << prefix << "    Effort: [";

  for (size_t ii = 0; ii < numDOFs; ii++)
  {
    buff << msg.effort[ii];
    if (ii < numDOFs - 1)
      buff << ", ";
  }

  buff << "]";

  return buff.str();
}

} // namespace utility
} // namespace controlit

#endif // _CONTROLIT_STRING_UTILITIES_HPP_
