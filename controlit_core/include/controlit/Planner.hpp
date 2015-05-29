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

#ifndef __CONTROLIT_CORE_PLANNER_HPP__
#define __CONTROLIT_CORE_PLANNER_HPP__

#include <string>
#include <yaml-cpp/yaml.h>

#include <controlit/ParameterReflection.hpp>
#include <controlit/Plan.hpp>
#include <sys/time.h>

namespace controlit {

/*!
 * The top-level class of all planners. Planners provide intermediate goals and
 * state transition commands to the controller.
 */
class Planner : public ParameterReflection
{
protected:

  // The constructors are protected because
  // a planner can only be instantiated via subclasses.

  /*!
   * The default constructor.
   */
  explicit Planner();

  /*!
   * The constructor.
   *
   * \param name The name of the task.
   */
  explicit Planner(std::string const& name);

public:

  /*!
   * Loads a task based on a YAML description.
   *
   * \param node The YAML node containing the task description.
   * \return Whether the task was successfully loaded.
   */
  bool loadConfig(YAML::Node const& node);

  /*!
   * Saves this task as a YAML description.
   *
   * \param node Where the YAML description should be saved.
   * \return Whether this task's description was successfully saved.
   */
  bool saveConfig(YAML::Emitter& node) const;

  /*!
   * Activates this planner.  This includes noting the start time,
   * which is the reference time from which the time offset is
   * computed when issuing the plan to the controller.
   */
  virtual void activate();

  /*!
   * Computes the amount of time that has passed since setStartTime was
   * called.
   *
   * \return The amount of time in seconds that have elapsed since
   * startStartTime was called.
   */
  double diffTime();

  /*!
   * This is periodically called by the PlannerExecutor after activate()
   * is called.  Based on the amount of time that has passed, it may
   * generate a plan or transmit an update of the intermediate goal states
   * to the controller.
   */
  virtual void update() = 0;

  /*!
   * Gets the plan. The plan consists of a sequence of intermediate goal states
   * and event commands.
   *
   * \param plan A reference to where the resulting plan should be saved.
   * \return Whether the plan was created successfully.
   */
  //virtual bool getPlan(Plan& plan) = 0;

  /*!
   * Prints the state of this planner.
   *
   * \param os The output stream to which to print.
   * \param prefix Text to print at the beginning of each section
   */
  virtual void dump(std::ostream& os, std::string const& prefix);

protected:
  /*!
   * Generates the plan so that it will be ready to execute.
   *
   * \return Whether the plan was successfully generated.
   */
  virtual bool generatePlan() = 0;

private:
  /*!
  * Adds the plan-specific parameters to the ParameterReflection infrastructure.
  */
  void setupParameters();

  /*!
   * The start time of this planner.  It is set when setStartTime() is called.
   */
  timeval startTime;
};

} // namespace controlit

#endif // __CONTROLIT_CORE_PLANNER_HPP__
