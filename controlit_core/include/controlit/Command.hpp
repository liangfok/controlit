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

#ifndef __CONTROLIT_CORE_COMMAND_HPP__
#define __CONTROLIT_CORE_COMMAND_HPP__

#include <controlit/addons/eigen/LinearAlgebra.hpp>

namespace controlit {

using controlit::addons::eigen::Vector;

/*!
 * A container class for the command produced by a WBC controller.
 */
class Command
{
public:
  /*!
   * The default constructor.
   */
  Command();

  /*!
   * A constructor that initializes the sizes of member variables.
   *
   * \param[in] jointNames The names and order of the joints in the command.
   */
  Command(const std::vector<std::string> & jointNames);

  /*!
   * The destructor.
   */
  virtual ~Command();

  /*!
   * Initializes this command object.
   *
   * \param jointNames the names and order of the joints being controlled.
   */
  bool init(const std::vector<std::string> & jointNames);

  /*!
   * Returns the number of DOFs in the command.
   * \note{This should only be called after init() is called.}
   */
  size_t getNumDOFs() const;

  /*!
   * Gets the effort command.
   *
   * \return the effort command.
   */
  Vector & getEffortCmd();

  /*!
   * Gets the position command.
   *
   * \return the position command.
   */
  Vector & getPositionCmd();

  /*!
   * Gets the velocity command.
   *
   * \return the velocity command.
   */
  Vector & getVelocityCmd();

  /*!
   * Gets the effective Kp gain for the joint position task.
   *
   * \return the effective Kp gain for the joint position task.
   */
  Vector & getEffectiveKp();

  /*!
   * Gets the effective Kd gain for the joint position task.
   *
   * \return the effective Kd gain for the joint position task.
   */
  Vector & getEffectiveKd();

  /*!
   * Gets the effort command.
   *
   * \return the effort command.
   */
  const Vector & getEffortCmd() const;

  /*!
   * Gets the position command.
   *
   * \return the position command.
   */
  const Vector & getPositionCmd() const;

  /*!
   * Gets the velocity command.
   *
   * \return the velocity command.
   */
  const Vector & getVelocityCmd() const;

  /*!
   * Gets the effective Kp gain for the joint position task.
   *
   * \return the effective Kp gain for the joint position task.
   */
  const Vector & getEffectiveKp() const;

  /*!
   * Gets the effective Kd gain for the joint position task.
   *
   * \return the effective Kd gain for the joint position task.
   */
  const Vector & getEffectiveKd() const;

  /*!
   * Dumps the state of this sensor set to an output stream.
   *
   * \param os The output stream.
   * \param prefix A prefix that is included at the beginning of each line
   * in the text dump.
   */
  void dump(std::ostream& os, std::string const & prefix) const;

  /*!
   * Returns a string representation of this class.
   */
  std::string toString(std::string const & prefix = "") const;

private:

  /*!
   * Whether init(...) was called.
   */
  bool initialized;

  std::vector<std::string> jointNames;

  Vector effortCmd;

  Vector positionCmd;

  Vector velocityCmd;

  Vector effectiveKp;

  Vector effectiveKd;
};

} // namespace controlit

#endif // __CONTROLIT_CORE_COMMAND_HPP__
