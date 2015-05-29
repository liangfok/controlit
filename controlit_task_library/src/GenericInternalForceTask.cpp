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

#include <controlit/task_library/GenericInternalForceTask.hpp>

namespace controlit {
namespace task_library  {

// Uncomment one of the following lines to enable/disable detailed debug statements.
#define PRINT_DEBUG_STATEMENT(ss)
// #define PRINT_DEBUG_STATEMENT(ss) CONTROLIT_PR_DEBUG << ss;

#define PRINT_DEBUG_STATEMENT_RT(ss)
// #define PRINT_DEBUG_STATEMENT_RT(ss) CONTROLIT_PR_DEBUG_RT << ss;


GenericInternalForceTask::GenericInternalForceTask() :
    Task("__UNNAMED_GENERIC_INTERNAL_FORCE_TASK__", controlit::CommandType::INTERNAL_FORCE,
        new TaskState(), new TaskState())
{
    setupParameters();
}

GenericInternalForceTask::GenericInternalForceTask(std::string const& name, const Vector & goalFint):
    Task(name, controlit::CommandType::INTERNAL_FORCE, new TaskState(), new TaskState()),
    goalFint_(goalFint)
{
    setupParameters();
}

void GenericInternalForceTask::setupParameters()
{
    paramGoalFint = declareParameter("goalFint", & goalFint_);
    paramActualFint = declareParameter("actualFint", & actualFint_);
    paramError = declareParameter("error", & error_);
    paramErrorDot = declareParameter("errorDot", & error_dot_);
}

bool GenericInternalForceTask::init(ControlModel & model)
{
    if (goalFint_.size() != model.virtualLinkageModel().getWint().rows())
    {
        CONTROLIT_WARN << "goalFint has incorrect length, expected " << model.virtualLinkageModel().getWint().rows()
                 << ", got " << goalFint_.size() << ", setting it equal to a zero vector of the correct length.";
        goalFint_.setZero(model.virtualLinkageModel().getWint().rows());
    }
  
    actualFint_.resize(goalFint_.size());
    actualFint.resize(goalFint_.size());
  
    // CONTROLIT_INFO << "Resizing goalFint_ to size " << model.virtualLinkageModel().getWint().rows();
    // controlit::utility::ContainerUtility containerUtility;
    // if (!containerUtility.checkMagnitude(goalFint_, 1e10))
    //   CONTROLIT_ERROR << "Error initializing goalFint_!\n"
    //             << "  - goalFint_ = " << goalFint_.transpose();
  
    return Task::init(model);
}

bool GenericInternalForceTask::updateStateImpl(ControlModel * model, TaskState * taskState)
{
    PRINT_DEBUG_STATEMENT("Method called!")
  
    assert(model != nullptr);
    assert(taskState != nullptr);
  
    // This should never be used...calculations done in controllerImpl.cpp
    // However, the size needs to be right; it needs to match the size of the command
    taskState->getJacobian().setZero(model->virtualLinkageModel().getWint().rows(), model->getNumDOFs());
  
    return true;
}

// bool GenericInternalForceTask::getJtask(ControlModel& model, Matrix & Jt)
// {
//   //This should never be used...calculations done in controllerImpl.cpp
//   //However, the size needs to be right; it needs to match the size of the command
//   Jt.setZero(model.virtualLinkageModel().getWint().rows(), model.getNumDOFs());
//   return true;
// }

bool GenericInternalForceTask::getCommand(ControlModel & model, TaskCommand & command)
{
    PRINT_DEBUG_STATEMENT_RT("Method called!")
  
    Matrix WintSensor;
    Vector FrSensor;
    model.virtualLinkageModel().getWintSensor(WintSensor);
    model.virtualLinkageModel().getFrSensor(FrSensor);
  
    actualFint = WintSensor * FrSensor;
    Vector errVec = goalFint_ - actualFint;
    double error = errVec.norm();
    double error_dot = abs(error - error_);
  
    // Publish parameters
    paramActualFint->set(actualFint);
    paramError->set(error);
    paramErrorDot->set(error_dot);
  
    // Set the command type
    command.type = commandType_;
  
    // Set the command
    command.command = goalFint_;
  
    return true;
}

} // namespace task_library
} // namespace controlit

