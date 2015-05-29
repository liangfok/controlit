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

#ifndef __CONTROLIT_CORE_CONTROLIT_PARAMETERS_HPP__
#define __CONTROLIT_CORE_CONTROLIT_PARAMETERS_HPP__

#include <vector>
#include "ros/ros.h"

#include <controlit/parser/Header.hpp>
#include <controlit/addons/eigen/LinearAlgebra.hpp>
#include <controlit/addons/ros/ROSParameterAccessor.hpp>

// service messages
#include <controlit_core/getControlItParameters.h>
#include <controlit_core/updateControlItParameters.h>

#include <diagnostic_msgs/DiagnosticStatus.h>

namespace controlit {
namespace utility {

using controlit::addons::ros::ROSParameterAccessor;
using controlit::addons::eigen::Vector;
using controlit::addons::eigen::Matrix;

/*!
 * Contains the WBC parameters.  These include torque offsets,
 * torque scaling factors, gravity, coupled joint groups, etc.
 */
class ControlItParameters
{
public:
    /*!
     * The constructor.
     */
    ControlItParameters();

    /*!
     * The destructor.
     */
    ~ControlItParameters();

    /*!
     * Initializes this class by loading the parameters from the ROS
     * parameter server.
     *
     * \param[in] nh The ROS node handle from which to obtain the WBC parameters.
     */
    bool init(ros::NodeHandle & nh);

    /*!
     * Perform some sanity checks on the parameters.
     * Ensure they have the right dimensions, etc.
     *
     * \param[in] numDOFs The number of actual degrees of freedom in the robot.
     * \return Whether the parameters are valid.
     */
    bool checkParameters();

    // bool enforceEffortLimits() { return enforceEffortLimits_; }
    // bool enforceVelocityLimits() { return enforceVelocityLimits_; }
    // bool enforceLowerPositionLimits() { return enforceLowerPositionLimits_; }
    // bool enforceUpperPositionLimits() { return enforceUpperPositionLimits_; }

    /*!
     * \return Whether the torque limits parameter is defined.
     */
    // bool hasTorqueLimits() { return hasTorqueLimits_; }

    /*!
     * \return Whether the velocity limits parameter is defined.
     */
    // bool hasVelocityLimits() { return hasVelocityLimits_; }

    /*!
     * \return Whether the upper position limits parameter is defined.
     */
    // bool hasPositionUpperLimits() { return hasPositionUpperLimits_; }

    /*!
     * \return Whether the lower position limits parameter is defined.
     */
    // bool hasPositionLowerLimits() { return hasPositionLowerLimits_; }

    /*!
     * \return Whether the torque offsets parameter is defined.
     */
    // bool hasTorqueOffsets() { return hasTorqueOffsets_; }

    /*!
     * \return Whether the torque scaling factors parameter is defined.
     */
    // bool hasTorqueScalingFactors() { return hasTorqueScalingFactors_; }

    /*!
     * \return Whether the ramp up time parameter is defined.
     */
    // bool hasRampUpTime() { return hasRampUpTime_; }

    /*!
     * \return Whether the coupled joint groups parameter is defined.
     */
    bool hasCoupledJointGroups() {return hasCoupledJointGroups_; }

    /*!
     * \return Whether the gravity compensation mask parameter is defined.
     */
    bool hasGravCompMask() {return hasGravCompMask_; }

    /*!
     * \return Whether the effective gain pass through mask parameter is defined.
     */
    // bool hasEffectiveGainPassThroughMask() { return hasEffectiveGainPassthroughMask_; }

    /*!
     * \return Whether the model base link parameter is defined.
     */
    bool hasModelBaseLinkName() { return hasModelBaseLinkName_; }

    /*!
     * \return Whether the rotor inertias are defined.
     */
    bool hasReflectedRotorInertias() { return hasReflectedRotorInertias_; }

    /*!
     * \return the target sevo frequency.
     */
    double getServoFrequency() { return servoFrequency; }
    /*!
     * \return The servo clock type.
     */
    std::string getServoClockType() { return servoClockType; }

    /*!
     * \return The robot interface type.
     */
    std::string getRobotInterfaceType() { return robotInterfaceType; }

    /*!
     * \return The controller type.
     */
    std::string getControllerType() { return controllerType; }

    /*!
     * \return Whether to use a single threaded control model
     */
    bool useSingleThreadedControlModel() { return useSingleThreadedControlModel_; }

    /*!
     * \return Whether to use a single threaded task updater
     */
    bool useSingleThreadedTaskUpdater() { return useSingleThreadedTaskUpdater_; }

    /*!
     * \return Whether to use a single threaded sensor updater
     */
    // bool useSingleThreadedSensorUpdater() { return useSingleThreadedSensorUpdater_; }

    /*!
     * \return the maximum effort command across any joint.
     */
    // double getMaxEffortCmd() { return maxEffortCmd; }

    /*!
     * \return The model integration cutoff delta.
     */
    // double getModelDeltaCutoff() { return modelDeltaCutoff; }

    /*!
     * \return The model blend rate.
     */
    // double getModelBlendRate() { return modelBlendRate; }

    /*!
     * \return The model's base link.
     */
    std::string getModelBaseLinkName() { return modelBaseLinkName; }

    /*!
     * \return The torque limits.
     */
    // const Vector & getTorqueLimits() { assert(hasTorqueLimits_); return torqueLimits; }

    /*!
     * \return The velocity limits.
     */
    // const Vector & getVelocityLimits() { assert(hasVelocityLimits_); return velocityLimits; }

    /*!
     * \return The upper position limits.
     */
    // const Vector & getPositionUpperLimits() { assert(hasPositionUpperLimits_); return positionUpperLimits; }

    /*!
     * \return The lower position limits.
     */
    // const Vector & getPositionLowerLimits() { assert(hasPositionLowerLimits_); return positionLowerLimits; }

    /*!
     * \return The torque offsets.
     */
    // const Vector & getTorqueOffsets() { return torqueOffsets; }

    /*!
     * \return The torque scaling factors.
     */
    // const Vector & getTorqueScalingFactors() { return torqueScalingFactors; }

    /*!
     * \return The gravity vector.  The units are m/s^2 in the x, y, z direction.
     */
    const Vector & getGravityVector() { return gravityVector; }

    /*!
     * \return The ramp up time.
     */
    // double getRampUpTime() { return rampUpTime; }

    /*!
     * \return the joint groups.
     */
    const std::vector<std::vector<std::string>> & getCoupledJointGroups() { return *coupledJointGroups; }

    /*!
     * \return the joints with no gravity compensation.
     */
    const std::vector<std::string> & getGravCompMask() { return *gravCompMask; }

    /*!
     * \return The which effective gains should be passed through without multiplication
     * with the inertia matrix.
     */
    // const std::vector<std::string> & getEffectiveGainPassThroughMask() { return *effectiveGainPassThroughMask; }

    /*!
     * \return The reflected rotor inertias.
     */
    const Vector & getReflectedRotorInertias() { return reflectedRotorInertias; }

    /*!
     * Save the joint limits.
     *
     * \param limits Contains the joint limit information.
     */
    // void saveLimits(WBCJointLimits &limits);

    /*!
     * Resize the member variables that store the joint limit information.
     *
     * \param size The number of actuable  DOFs in the robot.
     */
    // void resizeLimits(int size);

    /*!
     * Returns a string representation of the controller type.
     *
     * \param controllerType The controller type.
     * \return  A string representation of the controller type.
     */
    // std::string controllerTypeToString(ControllerType controllerType);

private:

    /*!
     * Loads the parameters.
     *
     * \param nh The node handle to use to get the parameters.
     */
    bool loadParameters(ros::NodeHandle & nh);

    /*!
     * Saves the WBC parameters into a diagnostic_msgs::DiagnosticStatus message.
     *
     * \param statusMsg The message into which the parameters should be saved.
     * \return true if successful.
     */
    bool saveParameters(diagnostic_msgs::DiagnosticStatus & statusMsg);

    // Methods for loading the WBC parameters
    bool loadServoClockType(ros::NodeHandle & nh);
    bool loadServoFrequency(ros::NodeHandle & nh);
    bool loadRobotInterfaceType(ros::NodeHandle & nh);
    bool loadControllerType(ros::NodeHandle & nh);

    bool loadControlModelSingleThreadedOption(ros::NodeHandle & nh);
    bool loadTaskUpdaterSingleThreadedOption(ros::NodeHandle & nh);
    // bool loadSingleThreadedSensorUpdater();
    bool loadUpdateRate(ros::NodeHandle & nh);
    bool loadMaxEffortCmd(ros::NodeHandle & nh);
    // bool loadTorqueLimits();
    // bool loadVelocityLimits();
    // bool loadPositionLowerLimits();
    // bool loadPositionUpperLimits();
    // bool loadTorqueOffsets();
    // bool loadTorqueScalingFactors();
    bool loadGravityVector();
    // bool loadRampUpTime();
    bool loadCoupledJointGroups();
    bool loadGravCompMask();
    // bool loadEffectiveGainPassThroughMask();
    bool loadOdometryTopic();
    // bool loadEnforceLimits();
    // bool loadModelDeltaCutoff();
    // bool loadModelBlendRate();
    bool loadModelBaseLink(ros::NodeHandle & nh);
    bool loadReflectedRotorInertias();

    // Service handler callback functions
    bool getParametersServiceHandler(
        controlit_core::getControlItParameters::Request  & req,
        controlit_core::getControlItParameters::Response & res);

    bool updateParametersServiceHandler(
        controlit_core::updateControlItParameters::Request  & req,
        controlit_core::updateControlItParameters::Response & res);

    /*!
     * The parameter interface is used to fetch parameters from
     * the ROS parameter server.
     */
    std::unique_ptr<ROSParameterAccessor> paramInterface;

    // Boolean guard variables that specify whether certain parameters
    // are defined within this object.
    // bool hasTorqueLimits_;
    // bool hasVelocityLimits_;
    // bool hasPositionLowerLimits_;
    // bool hasPositionUpperLimits_;
    // bool hasTorqueOffsets_;
    // bool hasTorqueScalingFactors_;
    // bool hasRampUpTime_;
    bool hasCoupledJointGroups_;
    bool hasGravCompMask_;
    // bool hasVelocityFilterAlpha_;
    // bool hasEffectiveGainPassthroughMask_;
    // bool hasModelBlendRate_;
    // bool hasModelDeltaCutoff_;
    bool hasModelBaseLinkName_;
    bool hasReflectedRotorInertias_;

    // Boolean variables for controlling whether limits are enforced by the coordinator
    // bool enforceEffortLimits_;
    // bool enforceVelocityLimits_;
    // bool enforceLowerPositionLimits_;
    // bool enforceUpperPositionLimits_;

    /*!
     * The type of the servo clock.
     */
    std::string servoClockType;

    /*!
     * The servo frequency.
     */
    double servoFrequency;

    /*!
     * The type of the robot interface.
     */
    std::string robotInterfaceType;

    /*!
     * The type of controller that should be used, e.g., TORQUE or IMPEDANCE
     */
    std::string controllerType;

    /*!
     * Whether to use a single threaded control model.
     */
    bool useSingleThreadedControlModel_;

    /*!
     * Whether to use a single threaded task updater.
     */
    bool useSingleThreadedTaskUpdater_;

    /*!
     * The gravity vector in m/s^2.  It should have a length of 3 (x, y, z).
     * By default it is (0, 0, -9.81).
     */
    Vector gravityVector;

    /*!
     * The coupled joint groups.  These are groups of joints that should feel coupling effects.
     */
    std::vector<std::vector<std::string>> * coupledJointGroups;

    /*!
     * The gravity compesation mask.  This is a list of joints that should not feel the effects
     * of gravity.
     */
    std::vector<std::string> * gravCompMask;

    /*!
     * The model base link.
     */
    std::string modelBaseLinkName;

    /*!
     * The rotor inertias.
     */
    Vector reflectedRotorInertias;

    /*!
     * The ROS service that provides details about the WBC parameters.
     */
    ros::ServiceServer getParametersService;

    /*!
     * The ROS service that results in the ControlItParameters being updated.
     */
    ros::ServiceServer updateParametersService;
};

} // namespace utility
} // namespace controlit

#endif // __CONTROLIT_CORE_CONTROLIT_PARAMETERS_HPP__