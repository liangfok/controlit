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

#ifndef __CONTROLIT_SERVO_CLOCK_LIBRARY_SERVO_CLOCK_CHRONO_HPP__
#define __CONTROLIT_SERVO_CLOCK_LIBRARY_SERVO_CLOCK_CHRONO_HPP__

#include <controlit/ServoClock.hpp>
#include <thread>  // for std::mutex

namespace controlit {
namespace servo_clock_library {

/*!
 * The coordinator for robots that are controlled via ROS topics.
 *
 * The robot_hardware_interface::ImpedanceCommandInterface type is
 * used as the template type for the Coordinator parent class but is
 * not actually used by this class.
 */
class ServoClockChrono : public controlit::ServoClock
{
public:
    /*!
     * The constructor.
     */
    ServoClockChrono();

    /*!
     * The destructor.
     */
    virtual ~ServoClockChrono();

protected:

    /*!
     * The implementation of the update loop.
     */
    virtual void updateLoopImpl();


};

} // namespace servo_clock_library
} // namespace controlit

#endif // __CONTROLIT_SERVO_CLOCK_LIBRARY_SERVO_CLOCK_CHRONO_HPP__
