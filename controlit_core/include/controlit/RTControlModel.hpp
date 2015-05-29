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

#ifndef __CONTROLIT_CORE_RT_CONTROL_MODEL_HPP__
#define __CONTROLIT_CORE_RT_CONTROL_MODEL_HPP__

#include <mutex>
#include <thread>
#include <condition_variable>
#include <chrono>

#include <ros/ros.h>

#include <controlit/ControlModel.hpp>
#include <controlit/Constraint.hpp>
#include <controlit/BindingManager.hpp>

#include <controlit/addons/ros/RealTimePublisher.hpp>

// Messages
#include <std_msgs/Float64.h>
#include <controlit_core/HolonomicConstraint.h>
#include "controlit_core/get_constraint_set_jacobians.h"

namespace controlit {

/*!
 * Instantiates two ControlModel objects and manages which
 * one is active.  The active ControlModel is used by the
 * main servo thread to compute the next command.  The inactive
 * ControlModel is used by a child thread to update the model
 * based on the current joint state.  The active and inactive
 * models are swapped when the child thread is done updating
 * the inactive model.
 */
class RTControlModel
{
public:
    /*!
     * The constructor.
     */
    RTControlModel();

    /*!
     * The destructor.
     */
    virtual ~RTControlModel();

    /*!
     * Whether this RTControlModel is initialized.
     */
    bool isInitialized() {return initialized;}

    /*!
     * Sets the robot control models to be stale.  This is to prevent
     * stale models from being used when the controller begins to run.
     */
    void setStale();

    /*!
     * Initialize this RTControlModel.
     *
     * \param[in] nh the ROS node handle.
     * \param[in] robotState A pointer to the latest robot state information.  This is
     * passed to the internal Control Model objects used within this object.
     * \param[in] parameterBindingManager The parameterBindingManager.
     * \param[out] params A pointer to the WBC parameters.  This is necessary to save the
     * joint limit information when the URDF is parsed.
     */
    bool init(ros::NodeHandle & nh, RobotState * robotState,
        BindingManager * parameterBindingManager,
        controlit::utility::ControlItParameters * params);

    /*!
     * Sets the gravity vector of all control models used internally
     * within this class.
     *
     * \param[in] gravityVector The gravity vector
     */
    virtual void setGravityVector(const Vector & gravityVector);

    /*!
     * Sets the A matrix mask of all control models used internally within this class.
     *
     * \param mask The A matrix mask.
     */
    virtual void setAMask(const std::vector<std::vector<std::string>> & mask);

    /*!
     * Sets the gravity mask of all control models used internally within this class.
     *
     * \param mask The gravity vector mask.
     */
    virtual void setGravMask(const std::vector<std::string> & mask);

    /*!
     * Adds a listener to the constraint set of every control model
     * used internally within this class.
     *
     * \param listener The listener.
     */
    virtual void addListenerToConstraintSet(boost::function<void(std::string const &)> listener);

    /*!
     * Starts the child thread that updates the inactive control model.
     */
    virtual void startThread();

    /*!
     * Stops the child thread that updates the inactive control model.
     */
    virtual void stopThread();

    /*!
     * Grabs the swap lock.  This prevents the active control model from being
     * swapped.  Note that this is a blocking operation and should *not*
     * be called by the RT thread!.
     */
    virtual void getSwapLock();

    /*!
     * Releases the swap lock.  This should only be called after calling getLock().
     * After calling it, the active control model may be swapped with the inactive
     * one.
     */
    virtual void releaseSwapLock();

    /*!
     * Attempt to grab the lock.  A pointer to the inactive ControlModel
     * will be returned if (1) the lock is successfully obtained,
     * (2) turn_ = REALTIME, and (3) newUpdateAvailable != true.
     *
     * \return A pointer to the inactive model if the lock is successfully
     * obtained, or nullptr otherwise.
     */
    virtual ControlModel * trylock();

    /*!
     * Releases the lock that protects the inactive model.
     */
    virtual void unlockAndUpdate();

    /*!
     * Check if an updated ControlModel is available.  If so upgrade
     * to it by swapping it and the currently-active ControlModel.
     *
     * \return true if a swap occurred.
     */
    virtual bool checkUpdate();

    /*!
     * Gets the active control model.
     *
     * \return A pointer to the active control model.
     */
    ControlModel* get();

    /*!
     * Returns the number of times the control model was updated.
     */
    // int getNumUpdates() {return numUpdates;}

protected:

    /*!
     * Define the possible states of the ModelUpdate thread.
     */
    enum class State : int {IDLE, UPDATING_MODEL, UPDATED_MODEL_READY};

    /*!
     * This is the state of the ModelUpdate thread.
     */
    State state;

    /*!
     * This is executed by the child thread that updates the inactive
     * control model.
     */
    void updateLoop();

    /*!
     * Swaps the inactive and active control models.
     */
    void swap();

    bool getConstraintJacobiansHandler(
        controlit_core::get_constraint_set_jacobians::Request  &req,
        controlit_core::get_constraint_set_jacobians::Response &res);

    /*!
     * Returns a string representation of the state.
     *
     * \param state The state.
     * \return A string description of the state.
     */
    std::string stateToString(State state) const;

    /*!
     * Whether the model update thread is running.
     */
    bool isRunning;

    /*!
     * Whether the model update thread should continue to run.
     */
    bool keepRunning;

    /*!
     * Whether init() was called.
     */
    bool initialized;

    /*!
     * The active control model.  This is used by the main servo thread
     * to compute the next command.
     */
    ControlModel * activeModel;

    /*!
     * The inactive control model.  This is used by a child thread to
     * update the model in parallel with the main servo thread.
     */
    ControlModel * inactiveModel;

    /*!
     * The parameter binding manager is used to create and destroy bindings.
     */
    BindingManager * parameterBindingManager;

    /*!
     * Notification of when it is OK to proceed with doing an update because state
     * information has been updated.
     */
    std::condition_variable cv;

    /*!
     * This mutex controls when the inactive control model can be updated.
     */
    std::mutex mutex;

    /*!
     * This mutex controls when the active ControlModel can be swapped with
     * the inactive one.
     */
    std::mutex swapMutex;

    /*!
     * This is the child thread that updates the inactive ControlModel.
     */
    std::thread thread;

    /*!
     * For publishing the model staleness information.
     */
    controlit::addons::ros::RealtimePublisher<std_msgs::Float64>
        modelUpdateLatencyPublisher;

    /*!
     * For publishing the gravity vector (diagnostics).
     */
    ros::Publisher gravityPublisher;

    /*!
     * The service that provides the constraint jacobian matrices.
     */
    ros::ServiceServer constraintJacobianService;

    /*!
     * A holonomic constraint.  This is used by method getConstraintJacobiansHandler().
     */
    controlit_core::HolonomicConstraint hc;

    /*!
     * The model update end time.
     */
    high_resolution_clock::time_point modelUpdateStartTime;

    /*!
     * The model update end time.
     */
    high_resolution_clock::time_point modelUpdateEndTime;
};

} // namespace controlit

#endif