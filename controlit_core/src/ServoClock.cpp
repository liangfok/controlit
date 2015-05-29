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

#include <controlit/ServoClock.hpp>

#include <controlit/logging/RealTimeLogging.hpp>

// Uncomment one of the following lines to enable/disable detailed debug statements.
#define PRINT_DEBUG_STATEMENT(ss)
// #define PRINT_DEBUG_STATEMENT(ss) CONTROLIT_DEBUG << ss;

#define PRINT_DEBUG_STATEMENT_RT(ss)
// #define PRINT_DEBUG_STATEMENT_RT(ss) CONTROLIT_DEBUG_RT << ss;

namespace controlit {

ServoClock::ServoClock() :
    callServoInit(true),
    continueRunning(false),
    isInitialized(false),
    isStarted(false)
{
    PRINT_DEBUG_STATEMENT("ServoClockCreated, isStarted = " << (isStarted ? "true" : "false"));
}

ServoClock::~ServoClock()
{
}

bool ServoClock::init(ServoableClass * servoableClass)
{
    if (!isInitialized)
    {
        this->servoableClass = servoableClass;
        isInitialized = true;
    }
    else
    {
        CONTROLIT_ERROR_RT << "Attempted to intialize twice.";
        return false;
    }

    return true;
}

bool ServoClock::start(double frequency)
{
    PRINT_DEBUG_STATEMENT("Method called! isStarted = " << (isStarted ? "true" : "false"));

    if (isInitialized)
    {
        if (!isStarted)
        {
            this->frequency = frequency;
            continueRunning = true;
            childThread = std::thread(&ServoClock::updateLoop, this);
            isStarted = true;
        }
        else
        {
            CONTROLIT_ERROR_RT << "Attempted to start multiple times.";
            return false;
        }
    }
    else
    {
        CONTROLIT_ERROR_RT << "Attempted to start without initializing.";
        return false;
    }

    return true;
}

bool ServoClock::stop()
{
    if (isInitialized && isStarted)
    {
        continueRunning = false;

        PRINT_DEBUG_STATEMENT("Waiting for child thread to join...");
        childThread.join();
        callServoInit = true;
        
        PRINT_DEBUG_STATEMENT("Done waiting for child thread.  ServoClock is stopped.");
        isStarted = false;
    }
    else
    {
        CONTROLIT_ERROR_RT << "Attempted to stop without initializing or clock was never started.";
        return false;
    }
    return true;
}

void ServoClock::updateLoop()
{
    updateLoopImpl();
}

} // namespace controlit
