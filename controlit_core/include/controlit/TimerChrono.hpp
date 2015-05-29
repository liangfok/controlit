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

#ifndef __CONTROLIT_CORE_TIMER_CHRONO_HPP__
#define __CONTROLIT_CORE_TIMER_CHRONO_HPP__

#include <chrono>
#include <controlit/Timer.hpp>

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;

namespace controlit {

class TimerChrono : public Timer
{
public:
    /*!
     * The default constructor.
     */
    explicit TimerChrono();
  
    /*!
     * Starts the timer.
     */
    virtual void start();

    /*!
     * Gets the timer's current value in seconds.
     *
     * \return The number of seconds that have elapsed since
     * the last call to start().
     */
    virtual double getTime();

private:
    high_resolution_clock::time_point startTime;
};

} // namespace controlit

#endif // __CONTROLIT_CORE_TIMER_CHRONO_HPP__

