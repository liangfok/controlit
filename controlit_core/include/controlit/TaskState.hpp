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

#ifndef __CONTROLIT_TASK_STATE_HPP__
#define __CONTROLIT_TASK_STATE_HPP__

#include <controlit/addons/eigen/LinearAlgebra.hpp>

namespace controlit {

using controlit::addons::eigen::Matrix;

/*!
 * Encapsulates the state of a task.
 */
class TaskState
{
public:

  /*!
   * The default constructor.
   */
  explicit TaskState();

  /*!
   * The destructor.
   */
  ~TaskState();

  /*!
   * An accessor the task's Jacobian matrix.
   *
   * \return A reference to the task's Jacobian matrix.
   */
  Matrix & getJacobian();

  /*!
   * Sets a flag indicating that the task Jacobian marix was set.
   */
  void setTaskJacobianFlag();

  /*!
   * Unsets a flag indicating that the task Jacobian marix was set.
   */
  void unsetTaskJacobianFlag();

  /*!
   * Whether the task jacobian was set.
   *
   * \return true if the task jacobian flag was set.
   */
  bool isTaskJacobianSet();

private:

  /*!
   * The task's Jacobian matrix.  This matrix converts from
   * the task's space into joint space.
   */
  Matrix taskJacobian;

  /*!
   * A flag for recalling whether the task Jacobian was set.
   */
  bool taskJacobianSet;

};

} // namespace controlit

#endif // __CONTROLIT_TASK_STATE_HPP__

