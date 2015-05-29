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

#include <controlit/Task.hpp>
#include <controlit/parser/yaml_parser.hpp>

namespace controlit {

// Uncomment the appropriate line below for enabling/disabling the printing of debug statements
#define PRINT_DEBUG_STATEMENT(ss)
// #define PRINT_DEBUG_STATEMENT(ss) CONTROLIT_PR_DEBUG << ss;

#define PRINT_DEBUG_STATEMENT_RT(ss)
// #define PRINT_DEBUG_STATEMENT_RT(ss) CONTROLIT_PR_DEBUG_RT << ss;

#define PRINT_ERROR_STATEMENT(ss) CONTROLIT_PR_ERROR << ss;

// #define PRINT_DEBUG_STATEMENT_RT_ALWAYS(ss) CONTROLIT_DEBUG_RT << ss;
#define PRINT_DEBUG_STATEMENT_RT_ALWAYS(ss) std::cout << ss << std::endl;

Task::Task() :
    PlanElement("Task", "__UNDEFINED_TASK_NAME__"),
    commandType_(controlit::ACCELERATION),
    tare(0),
    stateUpdateStatus(StateUpdateStatus::IDLE),
    initialized(false),
    inactiveState(nullptr),
    activeState(nullptr)
{
    setupParameters();
}

Task::Task(std::string const& typeName, CommandType commandType,
    TaskState * activeState, TaskState * inactiveState) :
    PlanElement("Task", typeName),
    commandType_(commandType),
    tare(0),
    stateUpdateStatus(StateUpdateStatus::IDLE),
    initialized(false),
    inactiveState(inactiveState),
    activeState(activeState)
{
    setupParameters();
}

Task::~Task()
{
    PRINT_DEBUG_STATEMENT("Method called!")
  
    if (activeState != nullptr)
    {
        PRINT_DEBUG_STATEMENT("Deleting active state.")
        delete activeState;
    }
  
    if (inactiveState != nullptr)
    {
        PRINT_DEBUG_STATEMENT("Deleting inactive state.")
        delete inactiveState;
    }
  
    PRINT_DEBUG_STATEMENT("Done method call.")
}

void Task::setupParameters()
{
    declareParameter("tare", &tare);  
}

bool Task::init(ControlModel & model)
{
    assert(!initialized);  // init can only be called once
    return Task::reinit(model);
}

bool Task::reinit(ControlModel & model)
{
    updateStateImpl(&model, inactiveState);
    updateStateImpl(&model, activeState);
  
    initialized = true;
  
    return true;
}

bool Task::loadConfig(YAML::Node const& node)
{
    node["type"] >> typeName;
    node["name"] >> instanceName;
    parse_parameter_reflection(node, static_cast<ParameterReflection*>(this));
    return true;
}

bool Task::saveConfig(YAML::Emitter& emitter) const
{
    emit_parameter_reflection(emitter, static_cast<ParameterReflection const*>(this));
    return true;
}

// This is called by a TaskUpdater thread after it receives an updated ControlModel
bool Task::updateState(ControlModel * model)
{
    PRINT_DEBUG_STATEMENT("Method called, stateUpdateStatus = " << stateUpdateStatusToString(stateUpdateStatus))
  
    if (stateUpdateStatus == StateUpdateStatus::IDLE)
    {
        stateUpdateStatus = StateUpdateStatus::UPDATING_STATE;
    
        // In the line below, updateStateImpl() is implemented by subclasses
        bool result = updateStateImpl(model, inactiveState);
    
        PRINT_DEBUG_STATEMENT("Changing stateUpdateStatus of task to be UPDATED_STATE_READY")
    
        // The following line tells the MainServo thread that this task has updated state.
        // See method checkUpdatedState() below.
        stateUpdateStatus = StateUpdateStatus::UPDATED_STATE_READY;
        return result;
    }
    else
    {
        PRINT_ERROR_STATEMENT("Attempted to update the state of a non-idle task!\n"
             " - stateUpdateStatus = " << stateUpdateStatusToString(stateUpdateStatus))
    
        return false;
    }
}

// This is called by the MainServo thread
bool Task::checkUpdatedState()
{
    PRINT_DEBUG_STATEMENT("Method called, stateUpdateStatus = " << stateUpdateStatusToString(stateUpdateStatus))
  
    // This is executed by the MainServo thread.  It checks if a new update is
    // available and if so swaps the active and inactive states.  Since this is
    // executed by the MainServo thread and the stateUpdateStatus will only be
    // UPDATED_STATE_READY after the ThreadUpdater thread is done, no need to
    // grab a lock on this object when performing the swap.
    if (stateUpdateStatus == StateUpdateStatus::UPDATED_STATE_READY)
    {
        PRINT_DEBUG_STATEMENT("Updated task state available, switching to it.")
    
        std::swap(activeState, inactiveState);
    
        PRINT_DEBUG_STATEMENT("Done updating task's state, setting stateUpdateStatus to be IDLE")
    
        stateUpdateStatus = StateUpdateStatus::IDLE;
    
        return true; // performed a swap
    }
    else
    {
        PRINT_DEBUG_STATEMENT("No new task state is available, sticking to current state.")
    }
  
    return false; // did not perform a swap
}

bool Task::getJacobian(Matrix & taskJacobian)
{
    PRINT_DEBUG_STATEMENT_RT("Method called!");
  
    // #define TIME_GET_TASK_JACOBIAN 1
  
    assert(activeState != nullptr);
  
    #ifdef TIME_GET_TASK_JACOBIAN
    ros::Time startSaveJacobian = ros::Time::now();
    #endif
  
    // WARNING!!  Enabling the log statement below will significantly increase the latency
    // of the servo loop!
    PRINT_DEBUG_STATEMENT_RT("Saving the active state's task Jacobian into parameter 'taskJacobian'.");
  
    taskJacobian = activeState->getJacobian();
  
    // WARNING!!  Enabling the log statement below will significantly increase the latency
    // of the servo loop!
    PRINT_DEBUG_STATEMENT_RT("The task's Jacobian matrix is:\n" << taskJacobian);
  
    #ifdef TIME_GET_TASK_JACOBIAN
    ros::Time endSaveJacobian = ros::Time::now();
    #endif
  
    #ifdef TIME_GET_TASK_JACOBIAN
      PRINT_DEBUG_STATEMENT_RT_ALWAYS("Task getJacobian Latency Results (ms) for task " << getInstanceName() << " (size: " << taskJacobian.rows() << ", " << taskJacobian.cols() << "):\n"
          " - saveJacobian: " << (endSaveJacobian - startSaveJacobian).toSec() * 1000);
    #endif
  
    return true;
}

std::string Task::stateUpdateStatusToString(StateUpdateStatus state)
{
    switch(state)
    {
        case StateUpdateStatus::IDLE: return "IDLE";
        case StateUpdateStatus::UPDATING_STATE: return "UPDATING_STATE";
        case StateUpdateStatus::UPDATED_STATE_READY: return "UPDATED_STATE_READY";
        default: return "UNKNOWN";
    }
}

void Task::dump(std::ostream& os, std::string const& title, std::string const& prefix) const
{
    ParameterReflection::dump(os, prefix + "  parameters", prefix + "    ");
}

} // namespace controlit
