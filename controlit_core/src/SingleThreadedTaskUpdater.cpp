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

#include <controlit/SingleThreadedTaskUpdater.hpp>

namespace controlit {

// Uncomment the appropriate lines below for enabling/disabling the printing of debug statements
#define PRINT_DEBUG_STATEMENT(ss)
// #define PRINT_DEBUG_STATEMENT(ss) CONTROLIT_DEBUG << ss;

#define PRINT_DEBUG_STATEMENT_RT(ss)
// #define PRINT_DEBUG_STATEMENT_RT(ss) CONTROLIT_DEBUG << ss;

#define PRINT_WARNING(ss) CONTROLIT_WARN << ss;
#define PRINT_WARNING_RT(ss) CONTROLIT_WARN_RT << ss;

SingleThreadedTaskUpdater::SingleThreadedTaskUpdater() //: controlit::TaskUpdater()
{
    PRINT_DEBUG_STATEMENT("Method Called!")
}

SingleThreadedTaskUpdater::~SingleThreadedTaskUpdater()
{
    PRINT_DEBUG_STATEMENT("Method Called!")
}

void SingleThreadedTaskUpdater::updateTasks(ControlModel * model)
{
    // In the single threaded version, go through and update the inactive state of
    // each task then swap the active and inactive states of the task.

    PRINT_DEBUG_STATEMENT_RT("Method called!")

    for(std::vector<Task *>::iterator it = taskSet.begin(); it != taskSet.end(); ++it)
    {
        Task * currTask = (*it);

        PRINT_DEBUG_STATEMENT("Updating the inactive state of task \""
            << currTask->getInstanceName() << "\", which is of type "
            << currTask->getTypeName())

        currTask->updateState(model);

        PRINT_DEBUG_STATEMENT("Swapping the inactive and active state of task \""
            << currTask->getInstanceName() << "\"")

        currTask->checkUpdatedState();
    }

    PRINT_DEBUG_STATEMENT("Done updating the tasks")
}

} // namespace controlit
