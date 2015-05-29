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

#ifndef __CONTROLIT_TASK_LIBRARY_COM_TASK_HPP__
#define __CONTROLIT_TASK_LIBRARY_COM_TASK_HPP__

#include <controlit/LatchedTask.hpp>
#include <controlit/task_library/PDController.hpp>
#include <controlit/addons/eigen/LinearAlgebra.hpp>

namespace controlit {
namespace task_library {

using controlit::addons::eigen::Vector;
using controlit::addons::eigen::Matrix;
using controlit::addons::eigen::Vector3d;
using controlit::addons::eigen::Matrix3d;

/*!
 * Joint-space posture task. Moves the joint positions towards a
 * desired posture using acceleration-bounded trajectories.
 *
 * \note Uses component-wise velocity saturation.
 * Parameters: inherited from PDTask.
 */
class COMTask : public controlit::LatchedTask
{
public:
  /*!
   * The default constructor.
   */
  COMTask();

  /*!
   * Initializes this task.
   *
   * \param[in] model The control model used to execute this task.
   */
  virtual bool init(ControlModel & model);

  /*!
   * Computes the desired commands.
   */
  virtual bool getCommand(ControlModel& model, TaskCommand & command);

protected:

  /*!
   * Overrides the super class' method.  An implementation of the updateState method.
   *
   * \param[in] model The robot's current active control model.
   * Note that this pointer should <b>not</b> be stored as a member
   * variable because it may not be the active one.
   * \param[in] taskState The TaskState that should be updated.
   * \return Whether the update state operation was successful.
   */
  virtual bool updateStateImpl(ControlModel * model, TaskState * taskState);

  Vector errpos, errvel;

  /*!
   * Goal position (un-projected) for the COM in the frameName_ frame...
   */
  Vector goalPosition_;

   /*!
   * Goal velocity (un-projected) for the COM in the frameName_ frame...
   */
  Vector goalVelocity_;

  /*!
   * PROJECTED position of the COM in the frameName_ frame
   */
  Vector frameProjectedCOM_;

  /*!
   * PROJECTED actual position of the COM in the world frame
   */
  Vector worldProjectedCOM_;

  /*!
   * PROJECTED velocity of the COM in the frameName_ frame
   */
  Vector frameProjectedCOMVel_;

  /*!
   * PROJECTED actual velocity of the COM in the world frame
   */
  Vector worldProjectedCOMVel_;

   /*!
   * Un-projected position of the COM in the world frame
   */
  Vector worldCOM_;

  /*!
   * Un-projected velocity of the COM in the world frame
   */
  Vector worldCOMVel_;

  /*!
   * A 3x3 matrix in the FRAME named by frameName_ parameter
   */
  Matrix projection_;

  /*!
   * A list of strings specifying the sub-robot to be used in this task.
   */
  std::vector<std::string> jointNameList_;

  /*!
   * A PD controller
   */
  std::unique_ptr<PDController> controller;

private:
  Matrix JvFrame, JwFrame, Jcom, JtLoc;
  Matrix RFrame;
  Vector TFrame;
  Vector VFrame;

  Matrix linkIndexMask;
  std::vector<unsigned int> linkIndexList;

  /*!
   * Persistent pointers to parameters.  These pointers are maintained to prevent
   * having to constantly call lookupParameter(...).
   */
  controlit::Parameter * paramFrameProjectedCOM;
  controlit::Parameter * paramFrameProjectedCOMVel;
  controlit::Parameter * paramWorldProjectedCOM;
  controlit::Parameter * paramWorldProjectedCOMVel;
  controlit::Parameter * paramWorldCOM;
  controlit::Parameter * paramWorldCOMVel;

};

} // namespace task_library
} // namespace controlit

#endif
