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

#ifndef __CONTROLIT_CORE_COMPOUND_TASK_HPP__
#define __CONTROLIT_CORE_COMPOUND_TASK_HPP__

#include <map>
#include <vector>

#include <controlit/addons/eigen/LinearAlgebra.hpp>
#include <controlit/ControlModel.hpp>
#include <controlit/TaskCommand.hpp>
#include <controlit/ReflectionRegistry.hpp>
#include <controlit/TaskUpdater.hpp>
#include <controlit/BindingManager.hpp>
#include <yaml-cpp/yaml.h>

namespace controlit {

// Forward declaration
class Task;
class TaskFactory;

/*!
 * A compound task consists of a set of prioritized tasks.
 * Multiple tasks may have the same priority.
 */
class CompoundTask : public ReflectionRegistry
{
public:
    /*!
     * The default constructor.
     */
    CompoundTask();
  
    /*!
     * A constructor.
     *
     * \param name The name of this compound task.
     */
    CompoundTask(std::string const& name);
  
    /*!
     * A set of tasks.
     * \todo: possibly switch to lists to improve performance
     */
    typedef std::vector<std::shared_ptr<Task>> TaskList_t;
  
    /*!
     * Each TaskList_t in the TaskTable_t is assumed to be of equal priority
     * For a purely hierarchical TaskTable_t, each entry should be a 1-dimensional TaskList_t.
     */
    typedef std::vector<TaskList_t> TaskTable_t;
  
    /*!
     * The destructor.
     */
    virtual ~CompoundTask();
  
    /*!
     * Initializes this compound task based on the specifications contained
     * within a YAML file.
     *
     * \param node The YAML node containing the compound task specification.
     */
    bool loadConfig(YAML::Node const & node);
  
    /*!
     * Generates a YAML specification based on the current state of this compound task.
     *
     * \param node This is where the results should be stored.
     * \return Whether the state of this compound task was successfully saved.
     */
    bool saveConfig(YAML::Emitter & node) const;
  
    /*!
     * Binds the parameters belonging to this compound task and all of the
     * tasks within it.
     *
     * \param[in] nh the ROS node handle.
     * \param[in] bm the binding manager.
     * \return Whether the binding of the parameters was successful.
     */
    virtual bool bindParameters(ros::NodeHandle & nh, BindingManager & bm);
  
    /*!
     * Initializes this compound task.  It initializes each task within this compound task.
     *
     * \param[in] model The model of the robot that is being controlled.
     */
    virtual bool init(ControlModel & model);
  
    /*!
     * Determines whether an internal force task is present.
     *
     * \return true if this compound task contains an internal force
     * task.
     */
    bool hasInternalForceTask() const { return hasIntForceTask; }
  
    /*!
     * Returns the row in the task table that holds the internal force task.
     *
     * NOTE 1: this can only ONE force task and cannot contain any other
     * acceleration tasks -- it is in the null space of all other tasks so
     * the priority is essentially meaningless, but we would like to use
     * the same storage structure
     *
     * NOTE 2: This method should only be called if hasInternalForceTask()
     * returns true.
     *
     * \return The priority level of the internal force task.
     */
    size_t getInternalForceTaskPriority() const { return intForceTaskPriority; }
  
    /*!
     * Adds a new task at a designated priority level
     * The priority level is 0-indexed
     *
     * \param task A pointer to the task being added.
     * \param priority The priority level of the task (0 is highest priority).
     */
    bool addTask(Task * task, size_t priority);
  
    /*!
     * Adds new task as lowest priority (emplace_back)
     *
     * \param task A pointer to the task being added.
     */
    bool addTask(Task * task);
  
    /*!
     * Computes the number of tasks within a task list that are enabled.
     *
     * \param taskList The list containing the tasks.
     * \param priorityLevel The priority level of the tasks in the list.
     * \return The number of tasks in the list that are enabled.
     */
    size_t numEnabledTasks(const std::vector<std::shared_ptr<Task>> & taskList, size_t priorityLevel) const;
  
    /*!
     * Adds the tasks within this compound task to the specified task updater.
     *
     * \param[in] taskUpdater A pointer to the task updater.
     * \return Whether the operation was successful.
     */
    bool addTasksToUpdater(TaskUpdater * taskUpdater);
  
    /*!
     * Queries whether the specified task is enabled.
     *
     * \param priorityLevel The prioritylevel of the task.
     * \param index The task's index within the priority level.
     * \return true if the task is enabled.
     */
    virtual bool isTaskEnabled(size_t priorityLevel, size_t index) const;
  
    // Here are a couple type defs used by getJacobianAndCommand()
    typedef std::vector<Matrix> TaskJacobians;
    typedef std::vector<Vector> TaskCommands;
    typedef std::vector<CommandType> TaskTypes;
  
    /*!
     * Obtains the compound task's Jacobian, command, and command type.
     *
     * \param[in] model The robot model.
     *
     * \param[out] Jt An aggregation of the task Jacobians of all tasks within this
     * compound task.  It consists of a std::vector of Matrix objects.
     * Each Matrix is the concatenation of the task Jacobians at a particular
     * priority level.
     *
     * \param command[out] An aggregation of the commands of all tasks within this
     * compound task.  It is a std::vector of Vector objects.  Each Vector is
     * the concatenation of the commands at a particular priority level.
     *
     * \param taskTypes[out] The type of the commands at each priority level.
     * All tasks at a particular priority level are assumed to be of the same type.
     *
     * \return Whether the method call was successful.
     */
    bool getJacobianAndCommand(ControlModel & model, TaskJacobians & Jt,
        TaskCommands & Command, TaskTypes & Type) const;
  
    /*!
     * Dumps the state of this CompoundTask into a string.
     *
     * \param os The output stream to which to write.
     * \param prefix Text that is placed at the beginning of each line.
     */
    void dump(std::ostream & os, std::string const & prefix) const;
  
private:
    /*!
     * A table containing the tasks within this compound task.  Note that tasks are double-booked.
     * They get stored as ParameterReflection objects within the ReflectionRegistry
     * (he parent class of CompoundTask) and they are stored as Task objects within this table.asd
     * This is to prevent needing to perform a cast in init()/update() and to efficiently capture the
     * priority of the tasks.
     */
    TaskTable_t taskTable;
  
    /*!
     * Whether this compound task contains an internal force task.
     */
    bool hasIntForceTask;
  
    /*!
     * The priority level of the internal force task.
     */
    size_t intForceTaskPriority;
  
    /*!
     * A managed pointer to the task factory used when initializing a compound task
     * based on a YAML file.  The task factory is created in this class' constructor.
     */
    std::unique_ptr<TaskFactory> taskFactory;
  
    /*!
     * The number of actuable DOFs.  This is used to convert the embedded kp/kd gains
     * from a scalar into a vector, if necessary.
     */
    int numActuableDOFs;
};

} // namespace controlit

#endif // __CONTROLIT_CORE_COMPOUND_TASK_HPP__
