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

#ifndef __CONTROLIT_CORE_SERVO_CLOCK_HPP__
#define __CONTROLIT_CORE_SERVO_CLOCK_HPP__

#include <mutex>
#include <thread>
#include <controlit/ServoableClass.hpp>

namespace controlit {

/*!
 * Defines the interface to be used by all ServoClock plugins.
 */
class ServoClock
{
public:
    /*!
     * The default constructor.
     */
    ServoClock();

    /*!
     * The destructor.
     */
    ~ServoClock();

    /*!
     * Abstract, implemented by subclasses in order to initialize the
     * ServoClock. This is important for stateful ServoClocks. The init()
     * method gets called when ServoClocks are enabled/disabled at runtime, so
     * subclasses should NOT assume that init() only gets called once
     * at startup.
     *
     * \param[in] servoableClass The servoableClass to periodically update.
     * \param[in] nh The ROS node handle.
     */
    bool init(ServoableClass * servoableClass);

    /*!
     * Starts the servo clock. It spawns a thread that calls updateLoop(), which
     * calls updateLoopImpl().
     *
     * \param[in] frequency The frequency at which the servo should execute.
     * \return Whether the servo clock was started.
     */
    virtual bool start(double frequency);

    /*!
     * Stops the servo clock.
     */
    virtual bool stop();

    /*!
     * The update loop.
     */
    void updateLoop();

protected:

    /*!
     * The implementation of the update loop. This method should not return
     * until stop() is called. During each round, it calls 
     * servoableClass->update().
     */
    virtual void updateLoopImpl() = 0;

    /*!
     * The servoableClass to periodically update.
     */
    ServoableClass * servoableClass;

    /*!
     * The servo frequency.
     */
    double frequency;

    /*!
     * Whether to call the ServoableClass' servoInit method.
     */
    bool callServoInit;

    /*!
     * Whether the child thread should continue running.
     */
    bool continueRunning;

private:
    /*!
     * Whether this an instantiation of this class is initialized.
     */
    bool isInitialized;

    /*!
     * Whether the clock is started.
     */
    bool isStarted;

    /*!
     * This mutex controls protects the continueRunning boolean variable.
     */
    std::mutex mutex;

    /*!
     * This is the child thread that updates the inactive ControlModel.
     */
    std::thread childThread;
};

} // namespace controlit

#endif // __CONTROLIT_CORE_SERVO_CLOCK_HPP__

