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

#ifndef __CONTROLIT_TASK_LIBRARY_JOINT_LIMIT_TASK_HPP_
#define __CONTROLIT_TASK_LIBRARY_JOINT_LIMIT_TASK_HPP_

#include <controlit/Task.hpp>
#include <controlit/ControlModel.hpp>
#include <controlit/task_library/PDController.hpp>
#include <vector>
#include <controlit/addons/eigen/LinearAlgebra.hpp>

namespace controlit {
namespace task_library {

using controlit::addons::eigen::Vector;
using controlit::addons::eigen::Matrix;
using controlit::addons::eigen::Vector3d;
using controlit::addons::eigen::Matrix3d;

class JointLimitTask : public controlit::Task
{
public:
    /*!
     * The default constructor.
     */
    JointLimitTask();
  
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
  
    void setActiveLimits(ControlModel& model);
  
    // parameters
    Vector upperStopRad_;
    Vector upperTriggerRad_;
    Vector lowerStopRad_;
    Vector lowerTriggerRad_;
  
    // utility variables
    std::vector<int> activeLimits;
    Vector errpos, errvel;
    Vector curkd, curkp, curMaxVel;
    Vector allKd, allKp, allMaxVel;
  
    bool gainStored;
  
    /*!
     * A PD controller
     */
    std::unique_ptr<PDController> controller;

private:
    /*!
     * Persistent pointers to parameters.  These pointers are maintained to prevent
     * having to call lookupParameter(...) every time getCommand(...) is called.
     */
    controlit::Parameter * paramKp;
    controlit::Parameter * paramKd;
    controlit::Parameter * paramMaxVel;

};

} // namespace task_library
} // namespace controlit

#endif // __CONTROLIT_TASK_LIBRARY_JOINT_LIMIT_TASK_HPP_
