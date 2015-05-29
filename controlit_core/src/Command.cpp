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

#include <controlit/Command.hpp>

namespace controlit {

Command::Command() :
  initialized(false)
{
}

Command::Command(const std::vector<std::string> & jointNames)
{
  init(jointNames);
}

Command::~Command()
{
}

bool Command::init(const std::vector<std::string> & jointNames)
{
  unsigned int numDOFs = jointNames.size();

  effortCmd.resize(numDOFs);    effortCmd.setZero();
  positionCmd.resize(numDOFs);  positionCmd.setZero();
  velocityCmd.resize(numDOFs);  velocityCmd.setZero();
  effectiveKp.resize(numDOFs);  effectiveKp.setZero();
  effectiveKd.resize(numDOFs);  effectiveKd.setZero();

  this->jointNames.resize(numDOFs);
  for (unsigned int ii = 0; ii < numDOFs; ii++)
  {
    this->jointNames[ii] = jointNames[ii];
  }

  initialized = true;

  return true;
}

size_t Command::getNumDOFs() const
{
  assert(initialized);
  return jointNames.size();
}

Vector & Command::getEffortCmd()
{
  assert(initialized);
  return effortCmd;
}

Vector & Command::getPositionCmd()
{
  assert(initialized);
  return positionCmd;
}

Vector & Command::getVelocityCmd()
{
  assert(initialized);
  return velocityCmd;
}

Vector & Command::getEffectiveKp()
{
  assert(initialized);
  return effectiveKp;
}

Vector& Command::getEffectiveKd()
{
  assert(initialized);
  return effectiveKd;
}

const Vector & Command::getEffortCmd() const
{
  assert(initialized);
  return effortCmd;
}

const Vector & Command::getPositionCmd() const
{
  assert(initialized);
  return positionCmd;
}

const Vector & Command::getVelocityCmd() const
{
  assert(initialized);
  return velocityCmd;
}

const Vector & Command::getEffectiveKp() const
{
  assert(initialized);
  return effectiveKp;
}

const Vector& Command::getEffectiveKd() const
{
  assert(initialized);
  return effectiveKd;
}

void Command::dump(std::ostream& os, std::string const& prefix) const
{
  os << toString(prefix);
}

std::string Command::toString(std::string const& prefix) const
{
  std::stringstream ss;
  ss << prefix << "Command details:" << std::endl;
  ss << prefix << "  initialized: " << (initialized ? "true" : "false") << std::endl;
  ss << prefix << "  numDOFs: " << jointNames.size() << std::endl;
  ss << prefix << "  joints: [";
  for (size_t ii = 0; ii < jointNames.size(); ii++)
  {
    ss << jointNames[ii];
    if (ii < jointNames.size() - 1)
      ss << ", ";
  }
  ss << "]" << std::endl;
  ss << prefix << "  effort: " << effortCmd.transpose() << std::endl;
  ss << prefix << "  position: " << positionCmd.transpose() << std::endl;
  ss << prefix << "  velocity: " << velocityCmd.transpose() << std::endl;
  ss << prefix << "  effectiveKp: " << effectiveKp.transpose() << std::endl;
  ss << prefix << "  effectiveKd: " << effectiveKd.transpose() << std::endl;
  return ss.str();
}

} // namespace controlit
