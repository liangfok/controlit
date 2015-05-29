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

#include <controlit/TaskUpdater.hpp>

#include <controlit/logging/RealTimeLogging.hpp>

namespace controlit {

// Uncomment the appropriate lines below for enabling/disabling the printing of debug statements
#define PRINT_DEBUG_STATEMENT(ss)
// #define PRINT_DEBUG_STATEMENT(ss) CONTROLIT_DEBUG << ss;

#define PRINT_DEBUG_STATEMENT_RT(ss)
// #define PRINT_DEBUG_STATEMENT_RT(ss) CONTROLIT_DEBUG << ss;

#define PRINT_WARNING(ss) CONTROLIT_WARN << ss;
#define PRINT_WARNING_RT(ss) CONTROLIT_WARN_RT << ss;

#define MAX_NUM_FAILURES 3 // The number of consecutive failures at obtaining the lock before a warning message is printed.

TaskUpdater::TaskUpdater() :
    state(State::IDLE),
    keepRunning(false),
    isRunning(false),
    numUpdates(0),
    failureCount(0)
{
}

TaskUpdater::~TaskUpdater()
{
    if (isRunning)
    {
        stopThread();
    }
}

void TaskUpdater::addTask(Task * task)
{
    PRINT_DEBUG_STATEMENT("Method Called!")

    taskSet.push_back(task);

    PRINT_DEBUG_STATEMENT("Done.")
}

void TaskUpdater::startThread()
{
    PRINT_DEBUG_STATEMENT("Method Called!")

    // Should we add a check on whether the thread is already running?

    keepRunning = true;
    thread = std::thread(&TaskUpdater::updateLoop, this);

    PRINT_DEBUG_STATEMENT("Done.")
}

void TaskUpdater::stopThread()
{
    PRINT_DEBUG_STATEMENT("Method Called")

    if (!isRunning)
    {
        PRINT_DEBUG_STATEMENT("Not running!")
        return;
    }

    PRINT_DEBUG_STATEMENT("Trying to get lock...")

    while (!mutex.try_lock())
    {
        #define MUTEX_WAIT_DURATION_MS 100
        PRINT_DEBUG_STATEMENT("Could not get lock on mutex, waiting for "
            << MUTEX_WAIT_DURATION_MS << " ms.")
        std::this_thread::sleep_for(
            std::chrono::milliseconds(MUTEX_WAIT_DURATION_MS));
    }

    PRINT_DEBUG_STATEMENT("Got the lock on mutex!")

    PRINT_DEBUG_STATEMENT("Setting keepRunning = false")

    keepRunning = false;

    PRINT_DEBUG_STATEMENT("Calling cv.notify_one().")

    cv.notify_one();  // So the publishing loop can exit

    PRINT_DEBUG_STATEMENT("Unlocking the mutex.")

    mutex.unlock();

    PRINT_DEBUG_STATEMENT("Calling thread.join().")

    thread.join();

    PRINT_DEBUG_STATEMENT("Done.")
}

void TaskUpdater::updateTasks(ControlModel * model)
{
    PRINT_DEBUG_STATEMENT_RT("Method called, state = " << stateToString(state))

    if (state == State::IDLE)
    {
        PRINT_DEBUG_STATEMENT_RT("State is IDLE, trying to grab lock.")

        if (mutex.try_lock())
        {
            PRINT_DEBUG_STATEMENT_RT("Obtained lock, saving the model.")

            failureCount = 0;

            this->model = model;

            PRINT_DEBUG_STATEMENT_RT("Setting state to be UPDATING_TASK_STATE.")

            state = State::UPDATING_TASK_STATE;

            PRINT_DEBUG_STATEMENT_RT("Calling notify_one() on the condition variable.")

            cv.notify_one();

            PRINT_DEBUG_STATEMENT_RT("Releasing lock.")

            mutex.unlock();
        }
        else
        {
            if (++failureCount >= MAX_NUM_FAILURES)
            {
                PRINT_WARNING_RT("Failed to obtain lock after " << MAX_NUM_FAILURES << " consecutive attempts. Not updating task states.")
            }
        }
    }
    else
    {
        PRINT_WARNING_RT("Attempted to start an update when an update was already in progress.")
    }
}

void TaskUpdater::checkTasksForUpdates()
{
    PRINT_DEBUG_STATEMENT_RT("Method called!")

    for(std::vector<Task *>::iterator it = taskSet.begin(); it != taskSet.end(); ++it)
    {
        Task * currTask = (*it);

        PRINT_DEBUG_STATEMENT_RT("Checking the following task for updates: instance name = \""
            << currTask->getInstanceName() << "\", type = " << currTask->getTypeName())

        if (currTask->checkUpdatedState())
        {
            PRINT_DEBUG_STATEMENT_RT("Updated task " << currTask->getInstanceName() << "'s state!")
        }
        else
        {
            PRINT_DEBUG_STATEMENT_RT("Did not update task " << currTask->getInstanceName() << "'s state!")
        }
    }
}

// This is executed by the TaskUpdater thread
void TaskUpdater::updateLoop()
{
    PRINT_DEBUG_STATEMENT("Method Called\n"
        " - std::this_thread::get_id = " << std::this_thread::get_id())

    isRunning = true;
    state = State::IDLE;

    // Grab the lock on the mutex.  The TaskUpdate thread will always have the lock
    // unless it is waiting on the condition variable (cv).
    std::unique_lock<std::mutex> lk(mutex);

    while (keepRunning)
    {
        PRINT_DEBUG_STATEMENT("Waiting on condition variable...")

        /*
         * Wait on the mutex (this thread will temporarily release the lock on the mutex
         * while waiting.  This enables another thread to lock the mutex
         * and call notify_one() or notify_all() on it, which will wake up this thread.
         * After waking up but before moving on from this method call, two conditions
         * must be met:
         *
         *  1. The boolean expression within the lambda function must evaluate to true
         *  2. This thread must re-obtain the lock on the mutex
         */
        cv.wait(lk, [this] { return (state == State::UPDATING_TASK_STATE) || (not keepRunning); });
        //cv.wait(lk, [&state, &keepRunning] { return (state == State::UPDATING_TASK_STATE) || (not keepRunning); });

        // We have everything we need to proceed with an update
        if (keepRunning)
        {
            PRINT_DEBUG_STATEMENT("Updating the inactive state of the tasks!")

            // #define TIME_TASK_STATE_UPDATE 1

            #ifdef TIME_TASK_STATE_UPDATE
            ros::Time startTaskUpdate = ros::Time::now();
            #endif

            for(std::vector<Task *>::iterator it = taskSet.begin(); it != taskSet.end(); ++it)
            {
                Task * currTask = (*it);

                PRINT_DEBUG_STATEMENT("Updating the inactive state of task \""
                    << currTask->getInstanceName() << "\", which is of type "
                    << currTask->getTypeName())

                currTask->updateState(model);
            }

            #ifdef TIME_TASK_STATE_UPDATE
            ros::Time stopTaskUpdate = ros::Time::now();
            #endif

            numUpdates++;

            PRINT_DEBUG_STATEMENT("Done updating the tasks, # updates = " << numUpdates << ".  "
                "Setting the status to be IDLE!")

            state = State::IDLE;

            #ifdef TIME_TASK_STATE_UPDATE
            CONTROLIT_INFO << "Latency Results (ms):\n"
                        " - taskUpdate: " << (stopTaskUpdate - startTaskUpdate).toSec() * 1000;
            #endif
        }
    }

    PRINT_DEBUG_STATEMENT("Stopping TaskUpdater::update() thread.  Number of updates: " << numUpdates);

    // Reset some local variables
    state = State::IDLE;
    numUpdates = 0;
    isRunning = false;
}

std::string TaskUpdater::stateToString(State state) const
{
    switch(state)
    {
        case State::IDLE: return "IDLE";
        case State::UPDATING_TASK_STATE: return "UPDATING_TASK_STATE";
        default: return "UNKNOWN";
    }
}

} // namespace controlit
