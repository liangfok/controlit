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

#ifndef __CONTROLIT_TASK_LIBRARY_JOINT_POSITIOIN_TASK_HPP__
#define __CONTROLIT_TASK_LIBRARY_JOINT_POSITIOIN_TASK_HPP__

#include <controlit/Task.hpp>
#include <controlit/ControlModel.hpp>
#include <controlit/task_library/PDController.hpp>
#include <controlit/addons/eigen/LinearAlgebra.hpp>

namespace controlit {
namespace task_library {

using controlit::addons::eigen::Vector;
using controlit::addons::eigen::Matrix;
using controlit::addons::eigen::Vector3d;
using controlit::addons::eigen::Matrix3d;

/*!
 * Joint-space task. Moves the joint positions towards a desired join state using 
 * acceleration-bounded trajectories.
 */
class JointPositionTask : public controlit::Task
{
public:
    /*!
     * The default constructor.
     */
    JointPositionTask();
  
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
  
    //Utilities
    Vector errpos, errvel;
  
    //Parameters
    Vector goalPosition, goalVelocity, actualPosition, actualVelocity, currentGoalPosition, goalAcceleration, currentGoalAcceleration;
  
    /*!
     * A PD controller
     */
    std::unique_ptr<PDController> controller;
  
    /*!
     * Persistent pointers to parameters.  These pointers are maintained to prevent
     * having to call lookupParameter(...) every time getCommand(...) is called.
     */
    controlit::Parameter * paramActualPos; // actual position of each joint
    controlit::Parameter * paramActualVel; // actual velocity of each joint
    controlit::Parameter * paramCurrentGoal; // the current goal positon
    controlit::Parameter * paramCurrentGoalAccel;
  
    // ============================================================================
    // BEGIN DREAMER SPECIFIC CODE!
    /*!
     * Specifies which joints should be included in the Jacobian and command.
     */
    // bool jointMask[16];
    // int numUnmaskedJoints;
    // Matrix maskedJacobian;
    // Vector fullCommand;
    // END DREAMER-SPECIFIC CODE!
    // ============================================================================
  
private:
    void addDefaultBindings();
};

} // namespace task_library
} // namespace controlit

#endif // __CONTROLIT_TASK_LIBRARY_JOINT_POSITIOIN_TASK_HPP__

