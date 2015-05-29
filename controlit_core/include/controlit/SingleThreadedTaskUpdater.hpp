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

#ifndef __CONTROLIT_SINGLE_THREADED_TASK_UPDATER_HPP__
#define __CONTROLIT_SINGLE_THREADED_TASK_UPDATER_HPP__

#include <controlit/TaskUpdater.hpp>

namespace controlit {

/*!
 * Extends TaskUpdater to be a version that is single threaded.
 */
class SingleThreadedTaskUpdater : public TaskUpdater
{
public:

  /*!
   * The default constructor.
   */
  SingleThreadedTaskUpdater();

  /*!
   * The destructor.
   */
  ~SingleThreadedTaskUpdater();

  /*!
   * Overrides parent class to do nothing.
   */
  virtual void startThread() {}

  /*!
   * Overrides parent class to do nothing.
   */
  virtual void stopThread() {}

  /*!
   * Updates the active state of all tasks.  This is called by the RT servo thread
   * each time the ControlModel changes.
   *
   * \param[in] model The new ControlModel.
   */
  virtual void updateTasks(ControlModel * model);

  /*!
   * Overrides parent class to do nothing.
   */
  virtual void checkTasksForUpdates() {}

};

} // namespace controlit

#endif // __CONTROLIT_SINGLE_THREADED_TASK_UPDATER_HPP__

