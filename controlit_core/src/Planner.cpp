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

#include <yaml-cpp/yaml.h>
#include <controlit/ParameterReflection.hpp>
#include <controlit/Planner.hpp>
#include <controlit/parser/yaml_parser.hpp>

namespace controlit {

Planner::Planner() :
  ParameterReflection("__UNDEFINED_TYPE__", "__UNDEFINED_NAME__")
{
  setupParameters();
}

Planner::Planner(std::string const& name) :
  ParameterReflection("planner", name)
{
  setupParameters();
}

void Planner::dump(std::ostream& os, std::string const& prefix)
{
  ParameterReflection::dump(os, prefix + "controlit::Planner:\n" + prefix + prefix + "parameters:\n", prefix + prefix);
}

void Planner::setupParameters()
{
  //declareParameter("foo", &bar);
}

bool Planner::loadConfig(YAML::Node const& node)
{
  node["type"] >> typeName;
  node["name"] >> instanceName;
  parse_parameter_reflection(node, static_cast<ParameterReflection*>(this));

  CONTROLIT_PR_INFO << "Load config complete";

  return true;
}

bool Planner::saveConfig(YAML::Emitter& emitter) const
{

  emit_parameter_reflection(emitter, static_cast<controlit::ParameterReflection const*>(this));

  CONTROLIT_PR_INFO << "Save config complete";

  return true;
}

// void Planner::generatePlan()
// {
//   std::cout << "Planner::generatePlan: Method called." << std::endl;
//   // TODO: This method should be pure virtual.
// }

void Planner::activate()
{
  gettimeofday(&startTime, NULL);
  // std::cout << "Planner::setStartTime: Set start time to be: " << startTime.tv_sec << "." << startTime.tv_usec << std::endl;

}

double Planner::diffTime()
{
  timeval endTime;
  gettimeofday(&endTime, NULL);

  long seconds  = endTime.tv_sec  - startTime.tv_sec;
  long useconds = endTime.tv_usec - startTime.tv_usec;

  double duration = seconds + useconds/1000000.0;

  return duration;
}

// void Planner::checkUpdateGoal()
// {
//  std::cout << "Planner::checkUpdateGoal: Method called." << std::endl;
//  // TODO: This method should be pure virtual.
// }

} // namespace controlit