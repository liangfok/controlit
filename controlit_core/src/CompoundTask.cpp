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

#include <limits>
#include <controlit/CompoundTask.hpp>
#include <controlit/Task.hpp>
#include <controlit/ControlModel.hpp>
#include <controlit/parser/yaml_parser.hpp>
#include <controlit/TaskFactory.hpp>
#include <controlit/BindingManager.hpp>
#include <ros/ros.h>

namespace controlit {

// Uncomment the appropriate line below for enabling/disabling the printing of debug statements
#define PRINT_DEBUG_STATEMENT(ss)
// #define PRINT_DEBUG_STATEMENT(ss) CONTROLIT_PR_DEBUG << ss;

#define PRINT_DEBUG_STATEMENT_RT(ss)
// #define PRINT_DEBUG_STATEMENT_RT(ss) CONTROLIT_PR_DEBUG_RT << ss;

// #define PRINT_DEBUG_STATEMENT_RT_ALWAYS(ss) CONTROLIT_DEBUG_RT << ss;
#define PRINT_DEBUG_STATEMENT_RT_ALWAYS(ss) std::cout << ss << std::endl;

CompoundTask::CompoundTask() :
    ReflectionRegistry("compound_task", "__UNNAMED_COMPOUND_TASK_INSTANCE__"),
    hasIntForceTask(false),
    intForceTaskPriority(0)
{
    taskFactory.reset(new TaskFactory);
}

CompoundTask::CompoundTask(std::string const& name) :
    ReflectionRegistry("compound_task", name),
    hasIntForceTask(false),
    intForceTaskPriority(0)
{
    taskFactory.reset(new TaskFactory);
}

CompoundTask::~CompoundTask()
{
    taskTable.clear(); // This is to prevent segfault when an instance of this class is destroyed.
    ReflectionRegistry::clearParameterCollection();
}

bool CompoundTask::loadConfig(YAML::Node const& doc)
{
    // Before anything, load all of the tasks in the 'tasks' section.
    // We will come back later and query the TaskFactory for the tasks
    // we need.
    std::map<std::string, Task*> taskList;
    YAML::Node const* tasksNode = doc.FindValue("tasks");
    if (tasksNode != NULL)
    {
        for (YAML::Iterator it = tasksNode->begin(); it != tasksNode->end(); ++it)
        {
            // peek into task to get its type and name
            std::string taskType;
            (*it)["type"] >> taskType;
      
            std::string taskName;
            (*it)["name"] >> taskName;
      
            PRINT_DEBUG_STATEMENT_RT("Creating task of type '" << taskType << "' called '" << taskName << "'");
      
            Task* task = taskFactory->loadFromYaml(*it);
      
            if (task != NULL)
            {
                PRINT_DEBUG_STATEMENT_RT("Done creating task '" << taskName << "'");
                taskList.insert(std::make_pair(task->getInstanceName(), task));
            }
            else
            {
                CONTROLIT_PR_ERROR_RT << "Factory failed to create task";
                return false;
            }
        }
    }
    else
    {
        CONTROLIT_PR_ERROR_RT << "'tasks' section in YAML config not found. Aborting load config.";
        return false;
    }
  
    // Load CompoundTask parameters
    YAML::Node const* ctNode = doc.FindValue("compound_task");
    if (ctNode == NULL)
    {
        CONTROLIT_PR_ERROR_RT << "'compound_task' section in YAML config not found. Aborting load config.";
        return false;
    }
    (*ctNode)["type"] >> typeName;
    (*ctNode)["name"] >> instanceName;
  
    // Load the task list
    YAML::Node const* tasksListNode = ctNode->FindValue("task_list");
    if (tasksListNode != NULL)
    {
        for (YAML::Iterator it = tasksListNode->begin(); it != tasksListNode->end(); ++it)
        {
            std::string name;
            unsigned int priority;
            int enabled;
      
            (*it)["name"] >> name;
            (*it)["priority"] >> priority;
            if ((*it).FindValue("enabled"))
            {
                (*it)["enabled"] >> enabled;
            }
            else
            {
                CONTROLIT_PR_INFO_RT << "No enabled specification found for task '" << name << "'. Assuming enabled = true.";
                enabled = 1;
            }
      
            auto tuple = taskList.find(name);
            if (tuple != taskList.end())
            {
                tuple->second->lookupParameter("enabled")->set(enabled);
                addTask(tuple->second, priority);
            }
            else
            {
                CONTROLIT_PR_ERROR_RT 
                    << "Requested task '" << name << "' in 'task_list' section but no task of that "
                    << "name was present in 'tasks' section. Typo?";
                return false;
            }
        }
    }
    else
    {
        CONTROLIT_PR_ERROR_RT << "'task_list' section in YAML config not found. Loaded tasks will not be associated with this compound task.";
        return false;
    }
  
    // Load the rest of my parameters
    parse_parameter_reflection(*ctNode, static_cast<ParameterReflection*>(this));
    return true;
}

bool CompoundTask::saveConfig(YAML::Emitter& emitter) const
{
    emitter << YAML::BeginMap
        << YAML::Key            << "tasks"
        << YAML::Value          << YAML::BeginSeq;
  
    for (auto const& tasks : taskTable)
        for (auto const& task : tasks) task->saveConfig(emitter);
  
    emitter << YAML::EndSeq
        << YAML::Key            << "compound_task"
        << YAML::Value;
  
    emit_parameter_reflection(emitter, static_cast<controlit::ParameterReflection const*>(this));
  
    CONTROLIT_PR_INFO_RT << "Save config complete";
    return true;
}

bool CompoundTask::bindParameters(ros::NodeHandle & nh, BindingManager & bm)
{
    if (!bm.bindParameters(nh, *this))
        return false;
  
    for (auto const& taskList : taskTable)
    {
        for (auto const& task : taskList)
        {
            if (!bm.bindParameters(nh, *(task.get())))
                return false;
        }
    }
    return true;
}

bool CompoundTask::init(ControlModel & model)
{
    // CONTROLIT_PR_DEBUG_RT << "Method called!";
  
    // For each priority level
    for (size_t priority = 0; priority < taskTable.size(); priority++)
    {
        // Get the list of tasks in the current priority level
        TaskList_t tasks = taskTable[priority];
    
        // For each task in the current priority level
        for (size_t index = 0; index < tasks.size(); index++)
        {
            // Get the current task
            std::shared_ptr<Task> task = tasks[index];
      
            // CONTROLIT_PR_DEBUG_RT << "Initializing task " << task->getInstanceName();
      
            // Check whether the task is an internal force task.  If it is:
            //   1. Ensure it is the only one
            //   2. Ensure there are no other tasks at the same priority level
            //   3. Record its priority level
            if (task->getCommandType() == CommandType::INTERNAL_FORCE)
            {
                if(hasIntForceTask) // there is already a force task
                {
                    CONTROLIT_PR_ERROR << "Attempting to add multiple internal force tasks!\n"
                      << " - Task instance name: " << task->getInstanceName() << "\n"
                      << " - Priority: " << priority << "\n"
                      << " - Index: " << index;
                    return false;
                }
        
                hasIntForceTask = true;
                intForceTaskPriority = priority;
            }
      
            // Initialize the task
            if (!task->init(model)) return false;
        }
    }
  
    PRINT_DEBUG_STATEMENT("Init complete")
  
    return true;
}

bool CompoundTask::addTask(Task * task)
{
    // Save the task in a managed pointer
    std::shared_ptr<Task> taskSharedPtr(task);

    // Inform the parent class of this new task is a parameter collection
    if (!addParameterCollection(taskSharedPtr))
    {
        CONTROLIT_PR_ERROR << "Failed to add task '" << task->getInstanceName() << "' as a parameter collection. Aborting add.";
        return false;
    }

    // Store this in our task table for faster access..
    taskTable.emplace_back(1, taskSharedPtr); //emplace_back calls constructor for TaskList_t object

    CONTROLIT_PR_INFO << "Added task '" << task->getInstanceName() << "' with priority " << taskTable.size() - 1;

    return true;
}

bool CompoundTask::addTask(Task * task, size_t priority)
{
    // Store task in a shared pointer.
    std::shared_ptr<Task> taskSharedPtr(task);
  
    // Increase task table size if necessary.
    size_t tableLength = taskTable.size();
    if(priority > tableLength)
    {
        for (size_t ii = 0; ii < priority - tableLength; ii++)
        {
            taskTable.push_back(TaskList_t());
        }
    }
  
    // Inform parent class that the new task is another parameter collection
    if (!addParameterCollection(taskSharedPtr))
    {
        CONTROLIT_PR_ERROR << "Failed to add task '" << task->getInstanceName() << "' as a parameter collection. Aborting add.";
        return false;
    }
  
    // Ensure we are not trying to add an internal force task
    // at the same priority level as an existing task.
    if(task->getCommandType() == CommandType::INTERNAL_FORCE && priority <= tableLength - 1)
    {
        CONTROLIT_PR_ERROR << "Failed to add internal force task '" << task->getInstanceName() << "' because it cannot be at the same priority as any other task.";
        return false;
    }
  
    PRINT_DEBUG_STATEMENT("Adding task to task table.\n"
         " - priority: " << priority << "\n"
         " - tableLength: " << tableLength);
  
    // Add the task to the table.
    if(tableLength > 0 && priority <= tableLength - 1) // the new task has equal priority to an existing task
        taskTable[priority].push_back(taskSharedPtr);
    else //adds as lowest priority task
        taskTable.emplace_back(1, taskSharedPtr);
  
    PRINT_DEBUG_STATEMENT("Added task '" << task->getInstanceName() << "' [priority: " << priority << "]");
  
    return true;
}

bool CompoundTask::isTaskEnabled(size_t priorityLevel, size_t index) const
{
    return taskTable[priorityLevel][index]->isEnabled();
}

size_t CompoundTask::numEnabledTasks(const std::vector<std::shared_ptr<Task>> & taskList,
    size_t priorityLevel) const
{
    size_t result = 0;
  
    for (auto& task : taskList)
    {
        if (task->isEnabled())
            result++;
    }
  
    // CONTROLIT_PR_INFO_RT << "Number of enabled priority " << priorityLevel << " tasks: " << result;
  
    return result;
}

bool CompoundTask::addTasksToUpdater(TaskUpdater * taskUpdater)
{
    // Go through each list of lists of tasks and add the tasks to the
    // task updater.
    for(auto& taskList : taskTable)
    {
        for (auto& task : taskList)
        {
            taskUpdater->addTask(task.get());
        }
    }
    return true;
}

bool CompoundTask::getJacobianAndCommand(ControlModel& model, TaskJacobians& Jt,
    TaskCommands& Command, TaskTypes& Type) const
{
    // #define TIME_COMPOUND_TASK_GET_JT_AND_COMMAND 1
  
    // Ensure the output vectors have the correct length
    size_t taskTableSize = taskTable.size();

    if (Jt.size() != taskTableSize)
    {
        // Jt.clear();
        Jt.resize(taskTableSize);
    }

    if (Command.size() != taskTableSize)
    {
        // Command.clear();
        Command.resize(taskTableSize);
    }

    if (Type.size() != taskTableSize)
    {
        // Type.clear();
        Type.resize(taskTableSize);
    }

    size_t priorityLevel = 0;  // Keeps track of which priority level we are working with.
  
    #ifdef TIME_COMPOUND_TASK_GET_JT_AND_COMMAND
    std::vector<ros::Time> latencyGetJacobianAndCommand;
    latencyGetJacobianAndCommand.push_back(ros::Time::now());
    #endif
  
    for(auto& taskList : taskTable) // For each priority level
    {
        if(!hasIntForceTask || priorityLevel != intForceTaskPriority)
        {
      
            // #define TIME_COMPOUND_TASK_GET_JT_AND_COMMAND_SPECIFIC_LEVEL 1
      
            #ifdef TIME_COMPOUND_TASK_GET_JT_AND_COMMAND_SPECIFIC_LEVEL
            ros::Time startGetNumEnabledTasks = ros::Time::now();
            #endif
      
            // Get the number of enabled tasks at the current priority level
            int numTasks = numEnabledTasks(taskList, priorityLevel);
      
            #ifdef TIME_COMPOUND_TASK_GET_JT_AND_COMMAND_SPECIFIC_LEVEL
            ros::Time startInitVars = ros::Time::now();
            #endif
      
            // Create cumulative force and jacobian data structures
            std::vector<TaskCommand> cumCmd;   cumCmd.resize(numTasks);
            std::vector<Matrix> cumJacobian;   cumJacobian.resize(numTasks);
      
            size_t numJacobianRows = 0;  // The total number of rows in the Jacobian matrix
            size_t cumIndx = 0; // Which position in the cumulative force and jacobian to which to write
      
            #ifdef TIME_COMPOUND_TASK_GET_JT_AND_COMMAND_SPECIFIC_LEVEL
            ros::Time startGetJacobianAndCommand = ros::Time::now();
            #endif
      
            size_t taskIndex = 0;
            // Go through each task in the taskList and, if it is enabled,
            // grab its command and jacobian.
            for (auto& task : taskList)
            {
                if (task->isEnabled())
                {
                    // #define TIME_COMPOUND_TASK_GET_JT_AND_COMMAND_SPECIFIC_TASK 1
          
                    #ifdef TIME_COMPOUND_TASK_GET_JT_AND_COMMAND_SPECIFIC_TASK
                    ros::Time startGetJacobian = ros::Time::now();
                    #endif
          
                    //NOTE: some TASKS will have incorrect calculations if the order of this changes!!!
                    if (!task->getJacobian(cumJacobian[cumIndx])) return false;
          
                    #ifdef TIME_COMPOUND_TASK_GET_JT_AND_COMMAND_SPECIFIC_TASK
                    ros::Time startGetTaskCommand = ros::Time::now();
                    #endif
          
                    if (!task->getCommand(model, cumCmd[cumIndx])) return false;
          
                    #ifdef TIME_COMPOUND_TASK_GET_JT_AND_COMMAND_SPECIFIC_TASK
                    ros::Time endGetTaskCommand = ros::Time::now();
          
                    if (priorityLevel == 1 && taskIndex == 0)
                    {
                        PRINT_DEBUG_STATEMENT_RT_ALWAYS("getJacobianAndCommand Priority 1 Task 0 Latency Results (ms):\n"
                          " - getJacobian: " << (startGetTaskCommand - startGetJacobian).toSec() * 1000 << "\n"
                          " - getTaskCommand: " << (endGetTaskCommand - startGetTaskCommand).toSec() * 1000);
                    }
                    #endif
          
          //          CONTROLIT_PR_INFO_RT << "State from Task " << cumIndx << ":\n"
          //               " - task jacobian:\n" << cumJacobian[cumIndx] << "\n"
          //               " - task command: " << cumCmd[cumIndx].command.transpose();
          
          
                    numJacobianRows += cumJacobian[cumIndx].rows();
                    cumIndx++;
                }
        
                taskIndex++;
            }
      
            #ifdef TIME_COMPOUND_TASK_GET_JT_AND_COMMAND_SPECIFIC_LEVEL
            ros::Time startResizeDestVars = ros::Time::now();
            #endif
      
            // std::cout<<"numJacobianRows = "<<numJacobianRows<<std::endl;
            Jt[priorityLevel].resize(numJacobianRows, model.getNumDOFs());
            Command[priorityLevel].resize(numJacobianRows);
      
            #ifdef TIME_COMPOUND_TASK_GET_JT_AND_COMMAND_SPECIFIC_LEVEL
            ros::Time startSaveResults = ros::Time::now();
            #endif
      
            if(numJacobianRows > 0)
            {
                size_t rowIndex = 0;
                for(size_t ii = 0; ii < cumJacobian.size(); ii++)
                {
                    size_t numRows = cumJacobian[ii].rows();
                    Jt[priorityLevel].block(rowIndex, 0, numRows, cumJacobian[ii].cols()) = cumJacobian[ii];
          
                    // CONTROLIT_PR_INFO_RT << "About to save command:\n"
                    //      " - rowIndex = " << rowIndex << "\n"
                    //      " - numRows = " << numRows << "\n"
                    //      " - ii = " << ii << "\n"
                    //      " - size of cumCmd[" << ii << "].command.size() = " << cumCmd[ii].command.size();
          
                    Command[priorityLevel].block(rowIndex, 0, numRows, 1) = cumCmd[ii].command;
                    Type[priorityLevel] = cumCmd[ii].type;
                    rowIndex += numRows;
                }
            }
      
            #ifdef TIME_COMPOUND_TASK_GET_JT_AND_COMMAND_SPECIFIC_LEVEL
            ros::Time endSaveResults = ros::Time::now();
      
            if (priorityLevel == 1)
            {
                PRINT_DEBUG_STATEMENT_RT_ALWAYS("getJacobianAndCommand Priority 1 Latency Results (ms):\n"
                  " - getNumEnabledTasks: " << (startInitVars - startGetNumEnabledTasks).toSec() * 1000 << "\n"
                  " - initVars: " << (startGetJacobianAndCommand - startInitVars).toSec() * 1000 << "\n"
                  " - getJacobianAndCommand: " << (startResizeDestVars - startGetJacobianAndCommand).toSec() * 1000 << "\n"
                  " - resizeDestVars: " << (startSaveResults - startResizeDestVars).toSec() * 1000 << "\n"
                  " - saveResults: " << (endSaveResults - startSaveResults).toSec() * 1000
                );
            }
            #endif
        }
        else if (hasIntForceTask && priorityLevel == intForceTaskPriority)
        {
            std::shared_ptr<Task> task = taskList[0];
      
            if (task->isEnabled())
            {
                Jt[intForceTaskPriority].resize(model.virtualLinkageModel().getWint().rows(), model.getNumDOFs());
                Command[intForceTaskPriority].resize(model.virtualLinkageModel().getWint().rows());
                TaskCommand intForceCommand;
                task->getJacobian(Jt[priorityLevel]);
                task->getCommand(model, intForceCommand);
                Command[intForceTaskPriority] = intForceCommand.command;
                Type[intForceTaskPriority] = intForceCommand.type;
            }
        }
    
        priorityLevel++;
        // taskListCount++;
    
        #ifdef TIME_COMPOUND_TASK_GET_JT_AND_COMMAND
        latencyGetJacobianAndCommand.push_back(ros::Time::now());
        #endif
    }
    
    #ifdef TIME_COMPOUND_TASK_GET_JT_AND_COMMAND
    std::stringstream msgBuff;
    for (size_t ii = 0; ii < latencyGetJacobianAndCommand.size() - 1; ii++)
    {
        msgBuff << " - getJacobianAndCommandPriority" << ii << ": " << (latencyGetJacobianAndCommand[ii + 1] - latencyGetJacobianAndCommand[ii]).toSec() * 1000;
        if (ii < latencyGetJacobianAndCommand.size() - 2)
            msgBuff << "\n";
    }
    PRINT_DEBUG_STATEMENT_RT_ALWAYS("CompoundTak getJacobianAndCommand Latency Results (ms):\n" << msgBuff.str());
    #endif
  
    // CONTROLIT_PR_INFO_RT << "Latency of method call: " << (ros::Time::now() - startMethodCall).toSec() * 1000;
  
    return true;
}

void CompoundTask::dump(std::ostream& os, std::string const& prefix) const
{
    os << prefix << "CompoundTask details:" << std::endl;
    os << prefix << "  Type: " << getTypeName() << std::endl;
    os << prefix << "  Name: " << getInstanceName() << std::endl;
    os << prefix << "  Tasks:" << std::endl;
  
    int priorityLevel = 0;
  
    for(auto& taskList : taskTable) // For each priority level
    {
        // Compute the number of enabled tasks
        size_t numTasks = numEnabledTasks(taskList, priorityLevel++);
    
        os << prefix << "    Priority: " << (priorityLevel-1)
          << ", # Tasks: " << taskList.size()
          << ", # Enabled: " << numTasks
          << std::endl;
    
        // Go through each task in the taskList and print its status
        for (auto& task : taskList)
        {    
            os << prefix << "      Task: Type: " << task->getTypeName() << ", InstanceName: " << task->getInstanceName();
      
            if (task->isEnabled())
                os << ", Status: ENABLED" << std::endl;
            else
                os << ", Status: DISABLED" << std::endl;
        }
    }
}

} // namespace controlit
