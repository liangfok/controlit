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

#ifndef __CONTROLIT_CORE_COORDINATOR_HPP__
#define __CONTROLIT_CORE_COORDINATOR_HPP__

#include <memory>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <controlit/addons/ros/RealTimePublisherHeader.hpp>

// Various factories
#include <controlit/BindingManager.hpp>
#include <controlit/CompoundTaskFactory.hpp>
#include <controlit/ConstraintSetFactory.hpp>
#include <controlit/ControllerFactory.hpp>
#include <controlit/ServoClockFactory.hpp>
#include <controlit/RobotInterfaceFactory.hpp>

// Core components
#include <controlit/ControlModel.hpp>
#include <controlit/Controller.hpp>
#include <controlit/RobotState.hpp>
#include <controlit/TaskUpdater.hpp>
#include <controlit/SingleThreadedTaskUpdater.hpp>

#include <controlit/utility/ContainerUtility.hpp>
#include <controlit/utility/LinkCOMPublisher.hpp>
#include <controlit/utility/ControlItParameters.hpp>

#include <controlit/ServoableClass.hpp>
#include <controlit/RTControlModel.hpp>
#include <controlit/SingleThreadedControlModel.hpp>
#include <controlit/EigenRealtimeBuffer.hpp>
#include <controlit/Diagnostics.hpp>
#include <controlit/DiagnosticsInfoProvider.hpp>

#include <controlit_core/get_inertia_matrix.h>
#include <controlit_core/getControllerConfig.h>


#include <chrono>


#include <tf/transform_broadcaster.h>

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;

namespace controlit {

class Diagnostics;  // forward declaration

/*!
 * Coordinates the gathering of robot state, execution of controller
 * computations, and transmission of the resulting command to the robot.
 */
class Coordinator :
    public DiagnosticsInfoProvider,
    public ServoableClass
{
public:

    /*!
     * The constructor.
     */
    Coordinator();

    /*!
     * The destructor.
     */
    ~Coordinator();

    /*!
     * Initializes this coordinator.
     */
    bool init();

    /*!
     * Starts the servo loop.
     */
    bool start();

    /*!
     * Stops the servo loop.
     */
    bool stop();

    /*!
     * Returns true if init() was called.
     *
     * \return whether this coordinator is initialized.
     */
    bool isInitialized() { return initialized; }

    /*!
     * Gets whether the coordinator is running.
     * A coordinator is running after starting() is called.
     * It stops running when stopping() is called.
     *
     * \return whether the controller is running.
     */
    bool isRunning() { return running; }

    /*!
     * Gets a string representation of task parameters used by this controller.
     * This is used by controlit::Diagnostics.
     *
     * \param[out] keys A reference to where the names of the parameters should be stored.
     * \param[out] values A reference to where the values of the parameters should be stored.
     * \return Whether the operation was successful.
     */
    bool getTaskParameters(std::vector<std::string> & keys, std::vector<std::string> & values);

    /*!
     * Gets a string representation of constraint parameters used by this controller.
     * This is used by controlit::Diagnostics.
     *
     * \param[out] keys A reference to where the names of the parameters should be stored.
     * \param[out] values A reference to where the values of the parameters should be stored.
     * \return Whether the operation was successful.
     */
    bool getConstraintParameters(std::vector<std::string> & keys, std::vector<std::string> & values);

    /*!
     * Returns a vector containing a list of actuable joints.  The order matches
     * that of the robot model.
     *
     * \return The names of the actuated joints.
     */
    const std::vector<std::string> & getActuatedJointNames() const;

    /*!
     * Returns a vector containing a list of real joints, regardless of whether they are slaves.
     * The order matches that of the robot model.
     *
     * \return The names of the real joints.
     */
    const std::vector<std::string> & getRealJointNames() const;

    /*!
     * Returns the most recent command issued.
     */
    const controlit::Command * getLastCommand() const { return & command; };

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // QUICK HACK TO ALLOW CONTROLLER TO TALK TO ROBOT VIA ROS TOPICS
    // void updateRobotState(const boost::shared_ptr<sensor_msgs::JointState const>& robotState);

protected:

    /*!
     * Adds an event listener.
     *
     * \param listener The listener to add.
     */
    void addEventListener(boost::function<void(std::string const&)> listener);

    /*!
     * The robot control model.
     */
    RTControlModel * model;

private:

    /************************************************************************
     *                                                                      *
     *     The following methods are used during initialization.            *
     *                                                                      *
     ************************************************************************/

    /*!
     * Loads the coordinator parameters from the ROS parameter server.
     *
     * \param nh The node handle to use to obtain the parameters.
     * \return false if a fatal errors occurs, true otherwise.
     */
    bool loadParameters(ros::NodeHandle & nh);

    /*!
     * Applies the coordinator parameters.  This includes setting the gravity
     * vector in the model, decoupling the 'A' matrix, and setting the gravity
     * mask. This is necessary because the model is not created when
     * 'loadControlItParameters' is called.
     */
    bool applyParameters();

    bool loadCompoundTask(ros::NodeHandle & nh);

    bool loadModel(ros::NodeHandle & nh);


    /************************************************************************
     *                                                                      *
     *           The following methods are used during runtime.             *
     *                                                                      *
     ************************************************************************/

    /*!
     * Called once the first time the servo clock is started.
     */
    void servoInit();

    /*!
     * Called each time the servo loop should execute.
     */
    void servoUpdate();

    /*!
     * Computes the command by passing the current stat to the controller.
     * Saves the command in variable 'command'.
     *
     * \param time The current time.
     */
    bool computeCommand();

    /*!
     * Sends the events for the event listeners.
     */
    bool emitEvents();

    /*!
     * Implements the following logic:
     *     Check tasks for updates
     *     If TaskUpdater is IDLE:
     *         If an updated control model is available:
     *             Switch to the updated model.
     */
    void checkForTaskAndModelUpdates();

    /*!
     * Force the updating of a control model. This is done during the call to servoInit().
     *
     * \param[in] controlModel A pointer to the control that to update.
     * \return Whether the update was successful.
     */
    bool forceUpdateControlModel(ControlModel * controlModel);

    /*!
     * Updates the inactive control model if it is available.
     */
    void updateModel();

    /*!
     * Prints the model details.  This includes a table with two columns:
     * (1) the name of the joint and (2) the joint's index within the RBDL
     * model.
     */
    void printModelDetails();

    bool getControllerConfigServiceHandler(
        controlit_core::getControllerConfig::Request & req,
        controlit_core::getControllerConfig::Response & res);


    /*!
     * Enforces the safety of the command.  This includes applying
     * the torque/position/velocity ramps and checking whether limits
     * are are met.
     *
     * \return false if there was a fatal error (meaning this cycle of
     * the servo loop should abort), true otherwise.
     */
    // bool enforceCommandSafety();

    /************************************************************************
     *                                                                      *
     *                   Here starts the local variables.                   *
     *                                                                      *
     ************************************************************************/

    /*!
     * A Timer for measuring the servo frequency.
     */
    std::shared_ptr<Timer> servoFreqTimer;

    /*!
     * A timer for measuring the internal latencies of a servo update.
     */
    std::shared_ptr<Timer> servoLatencyTimer;

    // For keeping track of servoUpdate() internal latencies
    double latencyRead, latencyPublishOdom, latencyModelUpdate,
           latencyComputeCmd, latencyEvents, latencyWrite, latencyServo;

    /*!
     * Real-time safe publisher for joint state messages.
     */
    controlit::addons::ros::RealtimePublisherHeader<sensor_msgs::JointState>
        jointStatePublisher;

    /*!
     * This is used for publishing the odometry transforms on topic /tf.
     * It connects the world to the robot.
     */
    tf::TransformBroadcaster tfBroadcaster;


    /*!
     * For publishing the model staleness information.
     */
    controlit::addons::ros::RealtimePublisher<std_msgs::Float64>
        modelStalenessPublisher;

    /*!
     * Real-time safe publisher for servo computational latency messages.
     */
    controlit::addons::ros::RealtimePublisher<std_msgs::Float64MultiArray>
        servoComputeLatencyPublisher;

    /*!
     * Real-time safe publisher for servo frequency messages.
     */
    controlit::addons::ros::RealtimePublisher<std_msgs::Float64>
        servoFrequencyPublisher;

    /*!
     * The time when the servo loop last executed in seconds
     */
    // double lastServoExecuteTime;

    /*!
     * Whether the controller is initialized.
     */
    bool initialized;

    /*!
     * Whether the controller is running.
     */
    bool running;

    /*!
     * The command to send to the actuators.
     */
    controlit::Command command;

    /*!
     * The latest joint state information.
     */
    controlit::RobotState latestRobotState;

    /*!
     * A pointer to the task updater.
     */
    controlit::TaskUpdater * taskUpdater;

    /*!
     * The diagnostics manager.
     */
    controlit::Diagnostics diagnostics;

    /*!
     * This is used to ensure the tau vector contains valid values.
     */
    controlit::utility::ContainerUtility containerUtility;

    /*!
     * This contains WBC parameters like joint limits,
     * torque offsets, torque scaling factors, etc.
     */
    controlit::utility::ControlItParameters controlitParameters;

    // Factories for loading constraint sets, compound tasks, and sensor sets
    controlit::ConstraintSetFactory constraintSetFactory;
    controlit::CompoundTaskFactory compoundTaskFactory;
    controlit::RobotInterfaceFactory robotInterfaceFactory;
    controlit::ServoClockFactory servoClockFactory;

    /*!
     * The servo clock that implements the thread that periodically calls update
     * on this coordinator.
     */
    std::unique_ptr<ServoClock> servoClock;

    /*!
     * The robot interface.
     */
    std::unique_ptr<controlit::RobotInterface> robotInterface;

    /*!
     * The compound task.
     */
    std::unique_ptr<controlit::CompoundTask> compoundTask;

    /*!
     * The controller.
     */
    std::unique_ptr<controlit::Controller> controller;

    /*!
     * The parameter binding manager.  This manages connections between parameters
     * and various transport layers.
     */
    BindingManager bindingManager;

    /*!
     * The factory that is used to create whole body controllers.
     */
    ControllerFactory controllerFactory;

    /*!
     * A service that returns the current controller configuration.
     */
    ros::ServiceServer getControllerConfigService;
};

} // namespace controlit

#endif // __CONTROLIT_CORE_COORDINATOR_HPP__
