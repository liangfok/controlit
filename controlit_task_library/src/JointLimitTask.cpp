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

#include <controlit/task_library/JointLimitTask.hpp>

namespace controlit {
namespace task_library {

// Uncomment one of the following lines to enable/disable detailed debug statements.
#define PRINT_DEBUG_STATEMENT(ss)
// #define PRINT_DEBUG_STATEMENT(ss) CONTROLIT_PR_DEBUG << ss;

#define PRINT_DEBUG_STATEMENT_RT(ss)
// #define PRINT_DEBUG_STATEMENT_RT(ss) CONTROLIT_PR_DEBUG_RT << ss;

JointLimitTask::JointLimitTask()
    : controlit::Task("__UNNAMED_JOINT_LIMIT_TASK__", CommandType::ACCELERATION, new TaskState(), new TaskState()),
    paramKp(NULL),
    paramKd(NULL),
    paramMaxVel(NULL)
{
    declareParameter("upperStopRad", &upperStopRad_);
    declareParameter("upperTriggerRad", &upperTriggerRad_);
    declareParameter("lowerStopRad", &lowerStopRad_);
    declareParameter("lowerTriggerRad", &lowerTriggerRad_);
  
    // Create the PD controller
    controller.reset(PDControllerFactory::create(SaturationPolicy::ComponentWiseVel));
  
    // Add controller parameters to this task
    controller->declareParameters(this);
  
    gainStored = false;
}

bool JointLimitTask::init(ControlModel & model)
{
    PRINT_DEBUG_STATEMENT("Method called!")
  
    int dofs = model.getNumDOFs();
    controller->resize(dofs);
  
    allKd.resize(dofs);
    allKp.resize(dofs);
    allMaxVel.resize(dofs);
  
    if (upperStopRad_.rows() != dofs)
    {
        CONTROLIT_ERROR << "Goal position must have " << dofs << " dimensions, got " << upperStopRad_.rows();
        return false;
    }
  
    if (upperTriggerRad_.rows() != dofs)
    {
        CONTROLIT_ERROR << "Goal position must have " << dofs << " dimensions, got " << upperTriggerRad_.rows();
        return false;
    }
  
    if (lowerStopRad_.rows() != dofs)
    {
        CONTROLIT_ERROR << "Goal position must have " << dofs << " dimensions, got " << lowerStopRad_.rows();
        return false;
    }
  
    if (lowerTriggerRad_.rows() != dofs)
    {
        CONTROLIT_ERROR << "Goal position must have " << dofs << " dimensions, got " << lowerTriggerRad_.rows();
        return false;
    }
  
    if(!gainStored)
    {
        allKd = *(this->lookupParameter("kd")->getVector());
        allKp = *(this->lookupParameter("kp")->getVector());
        allMaxVel = *(this->lookupParameter("maxVelocity")->getVector());
        gainStored = true;
    }
  
    paramKp = this->lookupParameter("kp");
    paramKd = this->lookupParameter("kd");
    paramMaxVel = this->lookupParameter("maxVelocity");
  
    return Task::init(model);
}

bool JointLimitTask::updateStateImpl(ControlModel * model, TaskState * taskState)
{
    PRINT_DEBUG_STATEMENT("Method called!")
  
    assert(model != nullptr);
    assert(taskState != nullptr);
  
    Matrix & taskJacobian = taskState->getJacobian();
  
    int numDOFs = model->getNumDOFs();
  
    setActiveLimits(*model);
  
    taskJacobian.resize(activeLimits.size(), numDOFs);
    taskJacobian.setZero();
  
    for(unsigned int i = 0; i < activeLimits.size(); i++)
        taskJacobian(i, activeLimits[i]) = 1.0;
  
    return true;
}

// The getCommand method is inhereted from PDTask, which is the parent class
bool JointLimitTask::getCommand(ControlModel& model, TaskCommand & u)
{
    // Get the latest joint state information
    Vector Q(model.getNumDOFs());
    Vector Qd(model.getNumDOFs());
  
    model.getLatestFullState(Q, Qd);
  
    setActiveLimits(model);
    int nActiveLimits = activeLimits.size();
    int jointIndex;
    controller->resize(nActiveLimits);
  
    // std::cout<<"There are "<<nActiveLimits<<" active joint limits"<<std::endl;
    errpos.resize(nActiveLimits);
    errvel.resize(nActiveLimits);
    curkp.resize(nActiveLimits);
    curkd.resize(nActiveLimits);
    curMaxVel.resize(nActiveLimits);
    u.command.resize(nActiveLimits);
  
    for(int i = 0; i < nActiveLimits; i++)
    {
        jointIndex = activeLimits[i];
    
        curkp[i] = allKp[jointIndex];
        curkd[i] = allKd[jointIndex];
        curMaxVel[i] = allMaxVel[jointIndex];
        if(model.getQ()[jointIndex] < lowerTriggerRad_[jointIndex])
        {
            errpos[i] = lowerStopRad_[jointIndex] - Q[jointIndex];
            errvel[i] = -Qd[jointIndex];
        }
        if(Q[jointIndex] > upperTriggerRad_[jointIndex])
        {
            errpos[i] = upperStopRad_[jointIndex] - Q[jointIndex];
            errvel[i] = -Qd[jointIndex];
        }
    }
  
    // Publish the parameters
    paramKp->set(curkp);
    paramKd->set(curkd);
    paramMaxVel->set(curMaxVel);
  
    // Set the command type
    u.type = commandType_;
  
    // Compute the command
    controller->computeCommand(errpos, errvel, u.command, this);
    return true;
}

/*!
 * Iterates through each joint and adds its index into
 * the activeLimits vector if its either above or below
 * the limit thresholds.
 */
void JointLimitTask::setActiveLimits(ControlModel& model)
{
    activeLimits.clear();
  
    for(int ii = 0; ii < model.getNumDOFs(); ii++)
    {
        if(model.getQ()[ii] < lowerTriggerRad_[ii])
            activeLimits.push_back(ii);
    
        if(model.getQ()[ii] > upperTriggerRad_[ii])
            activeLimits.push_back(ii);
    }
}

} // namespace task_library
} // namespace controlit