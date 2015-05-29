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

#ifndef __CONTROLIT_CORE__DIAGNOSTICS_INFO_PROVIDER_HPP__
#define __CONTROLIT_CORE__DIAGNOSTICS_INFO_PROVIDER_HPP__

#include <controlit/Command.hpp>
#include <controlit/addons/eigen/LinearAlgebra.hpp>

#include <controlit_core/HolonomicConstraint.h>

namespace controlit {

using controlit::addons::eigen::Vector;
using controlit::addons::eigen::Matrix;
using controlit::addons::eigen::Vector3d;
using controlit::addons::eigen::Matrix3d;

/*!
 * Defines the methods that should be provided for the diagnostic subsystem.
 */
class DiagnosticsInfoProvider
{
public:

  /*!
   * The destructor.
   */
  virtual ~DiagnosticsInfoProvider() {}

  /*!
   * Gets a string representation of task parameters used by this controller.
   * This is used by controlit::Diagnostics.
   *
   * \param[out] keys A reference to where the names of the parameters should be stored.
   * \param[out] values A reference to where the values of the parameters should be stored.
   * \return Whether the operation was successful.
   */
  virtual bool getTaskParameters(std::vector<std::string> & keys,
    std::vector<std::string> & values) = 0;

  /*!
   * Gets a string representation of constraint parameters used by this controller.
   * This is used by controlit::Diagnostics.
   *
   * \param[out] keys A reference to where the names of the parameters should be stored.
   * \param[out] values A reference to where the values of the parameters should be stored.
   * \return Whether the operation was successful.
   */
  virtual bool getConstraintParameters(std::vector<std::string> & keys,
    std::vector<std::string> & values) = 0;

  /*!
   * Returns a vector containing a list of actuable joints.  The order matches
   * that of the robot model.
   *
   * \return The names of the actuated joints.
   */
  virtual const std::vector<std::string> & getActuatedJointNames() const = 0;

  /*!
   * Returns a vector containing a list of real joints regardless of whether they
   * are slave joints.  The order matches that of the robot model.
   *
   * \return The names of the real joints.
   */
  virtual const std::vector<std::string> & getRealJointNames() const = 0;

  /*!
   * Returns the first command issued.
   */
  // virtual const controlit::Command * getFirstCommand() const = 0;

  /*!
   * Returns the most recent command issued.
   */
  virtual const controlit::Command * getLastCommand() const = 0;
};

} // namespace controlit

#endif
