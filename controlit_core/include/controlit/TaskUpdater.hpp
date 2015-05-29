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

#ifndef __CONTROLIT_TASK_UPDATER_HPP__
#define __CONTROLIT_TASK_UPDATER_HPP__

#include <controlit/Task.hpp>
#include <mutex>
#include <thread>
#include <condition_variable>

namespace controlit {

class TaskUpdater
{
public:
  /*!
   * Define the possible states of the TaskUpdate thread.
   */
  enum class State : int {IDLE, UPDATING_TASK_STATE};

  /*!
   * The default constructor.
   */
  TaskUpdater();

  /*!
   * The destructor.
   */
  virtual ~TaskUpdater();

  /*!
   * Adds a task to be updated by this TaskUpdater.
   */
  virtual void addTask(Task * task);

  /*!
   * Starts the child thread that updates the inactive states of the tasks.
   */
  virtual void startThread();

  /*!
   * Stops the child thread that updates the inactive states of the tasks
   */
  virtual void stopThread();

  /*!
   * Tells the TaskUpdater's thread to update the task states.
   * This should be called by the MainServo thread whenever the
   * ControlModel changes.
   *
   * \param[in] model The new ControlModel.
   */
  virtual void updateTasks(ControlModel * model);

  /*!
   * Checks each task for updates.  If a task has an update, switch to it.
   * This method should be called by the MainServo thread.
   */
  virtual void checkTasksForUpdates();

  /*!
   * Returns the state of this TaskUpdater's thread.
   */
  virtual State getState() { return state; }

protected:
  /*!
   * This is executed by the child thread that updates the inactive
   * states of the tasks.
   */
  void updateLoop();

  /*!
   * The control mode to use when updating the tasks.
   */
  ControlModel * model;

  /*!
   * Returns a string representation of the state.
   *
   * \param state The status.
   * \return A string description of the state.
   */
  std::string stateToString(State state) const;

  /*!
   * The state of this TaskUpdater.
   */
  State state;

  /*!
   * The set of tasks that are updated by this task updater.
   */
  std::vector<Task *> taskSet;

  /*!
   * Whether the child thread should continue to run.
   */
  bool keepRunning;

  /*!
   * Whether the child thread is running.
   */
  bool isRunning;

  /*!
   * This mutex ensures a smooth transision between the MainServo thread
   * and the TaskUpdater thread.
   */
  std::mutex mutex;

  /*!
   * Notification when its ok proceed with doing an update because state
   * information has been updated.
   */
  std::condition_variable cv;

  /*!
   * This is the child thread that updates the tasks' states.
   */
  std::thread thread;

  /*!
   * The number of times the tasks were updated.
   */
  size_t numUpdates;

  /*!
   * The number of consecutive times the task updater failed to grab the lock
   * and thus not update any tasks.
   */
  size_t failureCount;

};

} // namespace controlit

#endif // __CONTROLIT_TASK_UPDATER_HPP__

