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

#include <controlit/utility/ControlItParameters.hpp>

#include <controlit/addons/ros/ROSParameterAccessor.hpp>
#include <controlit/logging/RealTimeLogging.hpp>
#include <boost/lexical_cast.hpp>

namespace controlit {
namespace utility {

// Uncomment one of the following lines to enable/disable detailed debug statements.
// #define PRINT_DEBUG_STATEMENT(ss)
#define PRINT_DEBUG_STATEMENT(ss)  CONTROLIT_DEBUG << ss;


#define PARAM_SERVO_CLOCK_TYPE                  "controlit/servo_clock_type"
#define PARAM_SERVO_FREQUENCY                   "controlit/servo_frequency"
#define PARAM_ROBOT_INTERFACE_TYPE              "controlit/robot_interface_type"
#define PARAM_WBC_CONTROLLER_TYPE               "controlit/whole_body_controller_type"
#define PARAM_USE_SINGLE_THREADED_CONTROL_MODEL "controlit/use_single_threaded_control_model"
#define PARAM_USE_SINGLE_THREADED_TASK_UPDATER  "controlit/use_single_threaded_task_updater"
#define PARAM_GRAVITY_VECTOR                    "controlit/gravity_vector"
#define PARAM_COUPLED_JOINT_GROUPS              "controlit/coupled_joint_groups"
#define PARAM_GRAVITY_COMP_MASK                 "controlit/gravity_compensation_mask"
#define PARAM_MODEL_BASE_LINK_NAME              "controlit/model_base_link_name"

// #define TORQUE_OFFSETS_PARAMETER "controlit/torque_offsets"
// #define TORQUE_SCALING_FACTORS   "controlit/torque_scaling_factors"

ControlItParameters::ControlItParameters() :
    // hasTorqueLimits_(false),
    // hasVelocityLimits_(false),
    // hasPositionLowerLimits_(false),
    // hasPositionUpperLimits_(false),
    // hasTorqueOffsets_(false),
    // hasTorqueScalingFactors_(false),
    // hasRampUpTime_(true),
    hasCoupledJointGroups_(false),
    hasGravCompMask_(false),
    // hasVelocityFilterAlpha_(false),
    // hasEffectiveGainPassthroughMask_(false),
    // hasMixingParameter_(false),
    // hasModelBlendRate_(false),
    // hasModelDeltaCutoff_(false),
    hasModelBaseLinkName_(false),
    hasReflectedRotorInertias_(false),
  
    // By default do not enforce any limits
    // enforceEffortLimits_(false),
    // enforceVelocityLimits_(false),
    // enforceLowerPositionLimits_(false),
    // enforceUpperPositionLimits_(false),
  
    servoClockType("controlit_servo_clock/ServoClockROS"),
    servoFrequency(1000),
    robotInterfaceType("controlit_robot_interface/RobotInterfaceSM"),
    controllerType("controlit_wbc/WBOSC"),
  
    useSingleThreadedControlModel_(false),
    useSingleThreadedTaskUpdater_(false),
    // useSingleThreadedSensorUpdater_(false),
  
    // maxEffortCmd(1e4),  // any effort command above 1e4 is considered invalid
    // modelBlendRate(0.9),
    // modelDeltaCutoff(0.03),
    // rampUpTime(0), // ramp up time is by default zero
    coupledJointGroups(nullptr),
    gravCompMask(nullptr)
    // effectiveGainPassThroughMask(nullptr)
{
    // By default the gravity vector is x=0, y=0, z=-9.81 m/s^2
    gravityVector.setZero(3);
    gravityVector(2) = -9.81;
}

ControlItParameters::~ControlItParameters()
{
    if (coupledJointGroups != NULL) delete coupledJointGroups;
    if (gravCompMask != NULL) delete gravCompMask;
}

bool ControlItParameters::init(ros::NodeHandle & nh)
{

    this->paramInterface.reset(new controlit::addons::ros::ROSParameterAccessor(nh));

    // Load the parameters
    bool result = loadParameters(nh);

    // Advertise a service that allows the user to query the WBC parameters
    getParametersService = nh.advertiseService("diagnostics/getControlItParameters",
        &ControlItParameters::getParametersServiceHandler, this);

    // Advertise a service that forces the WBC parameters to be updated based on
    // the values in the ROS parameter server
    updateParametersService = nh.advertiseService("controlit/updateControlItParameters",
        &ControlItParameters::updateParametersServiceHandler, this);

    return result;
}

bool ControlItParameters::checkParameters()
{
    // assert(paramInterface != nullptr);

    if (gravityVector.size() != 3)
    {
        CONTROLIT_ERROR 
            << "ROS parameter '" << paramInterface->getNamespace() << "/" << PARAM_GRAVITY_VECTOR << "' contains "
            << "value of incorrect length.  Expected 3 got " << gravityVector.size() << ".";
        return false;
    }

    return true;
}

bool ControlItParameters::loadParameters(ros::NodeHandle & nh)
{
    assert(paramInterface != nullptr);

    if (!loadServoClockType(nh)) return false;
    if (!loadServoFrequency(nh)) return false;
    if (!loadRobotInterfaceType(nh)) return false;
    if (!loadControllerType(nh)) return false;
    if (!loadControlModelSingleThreadedOption(nh)) return false;
    if (!loadTaskUpdaterSingleThreadedOption(nh)) return false;
    // if (!loadMaxEffortCmd(nh)) return false;
    // if (!loadTorqueOffsets(nh)) return false;
    // if (!loadTorqueScalingFactors(nh)) return false;
    if (!loadGravityVector()) return false;
    if (!loadCoupledJointGroups()) return false;
    if (!loadGravCompMask()) return false;
    // if (!loadEnforceLimits(nh)) return false;
    if (!loadModelBaseLink(nh)) return false;
    if (!loadReflectedRotorInertias()) return false;

    return true;
}

bool ControlItParameters::loadServoClockType(ros::NodeHandle & nh)
{
    if (!nh.getParam(PARAM_SERVO_CLOCK_TYPE, servoClockType))
    {
        CONTROLIT_ERROR 
            << "Servo clock type not specified on ROS parameter server.  "
            << "Ensure parameter \"" << paramInterface->getNamespace() << "/" << PARAM_SERVO_CLOCK_TYPE << "\" is specified";
        return false;
    }
    return true;
}

bool ControlItParameters::loadServoFrequency(ros::NodeHandle & nh)
{
    if (!nh.getParam(PARAM_SERVO_FREQUENCY, servoFrequency))
    {
        CONTROLIT_ERROR 
            << "Servo frequency not specified on ROS parameter server.  "
            << "Ensure parameter \"" << paramInterface->getNamespace() << "/" << PARAM_SERVO_FREQUENCY << "\" is specified";
        return false;
    }
    return true;
}

bool ControlItParameters::loadRobotInterfaceType(ros::NodeHandle & nh)
{
    if (!nh.getParam(PARAM_ROBOT_INTERFACE_TYPE, robotInterfaceType))
    {
        CONTROLIT_ERROR 
            << "Robot interface not specified on ROS parameter server.  "
            << "Ensure parameter \"" << paramInterface->getNamespace() << "/" << PARAM_ROBOT_INTERFACE_TYPE << "\" is specified";
        return false;
    }
    return true;
}

bool ControlItParameters::loadControllerType(ros::NodeHandle & nh)
{
    if (!nh.getParam(PARAM_WBC_CONTROLLER_TYPE, controllerType))
    {
        CONTROLIT_ERROR 
            << "Controller type not specified on ROS parameter server.  "
            << "Ensure parameter \"" << paramInterface->getNamespace() << "/" << PARAM_WBC_CONTROLLER_TYPE << "\" is specified";
        return false;
    }
    else
    {
        return true; 
    }
}

bool ControlItParameters::loadControlModelSingleThreadedOption(ros::NodeHandle & nh)
{
    nh.getParam(PARAM_USE_SINGLE_THREADED_CONTROL_MODEL, useSingleThreadedControlModel_);
    return true;
}

bool ControlItParameters::loadTaskUpdaterSingleThreadedOption(ros::NodeHandle & nh)
{
    nh.getParam(PARAM_USE_SINGLE_THREADED_TASK_UPDATER, useSingleThreadedTaskUpdater_);
    return true;
}

bool ControlItParameters::loadGravityVector()
{
    paramInterface->loadParameter(PARAM_GRAVITY_VECTOR, gravityVector);
    return true;
}

bool ControlItParameters::loadCoupledJointGroups()
{
    hasCoupledJointGroups_ = paramInterface->loadParameter(PARAM_COUPLED_JOINT_GROUPS, &coupledJointGroups);
    return true;
}

bool ControlItParameters::loadGravCompMask()
{
    hasGravCompMask_ = paramInterface->loadParameter(PARAM_GRAVITY_COMP_MASK, &gravCompMask);
    return true;
}

bool ControlItParameters::loadModelBaseLink(ros::NodeHandle & nh)
{
    hasModelBaseLinkName_ = nh.getParam(PARAM_MODEL_BASE_LINK_NAME, modelBaseLinkName);
    return true;
}

// bool ControlItParameters::loadMaxEffortCmd()
// {
//     paramInterface->loadParameter("controlit/max_effort_command", maxEffortCmd);
//     return true;
// }

// bool ControlItParameters::loadTorqueOffsets()
// {
//     hasTorqueOffsets_ = paramInterface->loadParameter(TORQUE_OFFSETS_PARAMETER, torqueOffsets);
//     return true;
// }

// bool ControlItParameters::loadTorqueScalingFactors()
// {
//     hasTorqueScalingFactors_ = paramInterface->loadParameter(TORQUE_SCALING_FACTORS, torqueScalingFactors);
//     return true;
// }


// bool ControlItParameters::loadRampUpTime()
// {
//     hasRampUpTime_ = paramInterface->loadParameter("controlit_ramp_up_time", rampUpTime);
//     return true;
// }



// bool ControlItParameters::loadEffectiveGainPassThroughMask()
// {
//     hasEffectiveGainPassthroughMask_ = paramInterface->loadParameter("controlit_effective_gain_pass_through_mask", &effectiveGainPassThroughMask);
//     return true;
// }



// bool ControlItParameters::loadEnforceLimits()
// {
//     // paramInterface->loadParameter("enforce_effort_limits",         enforceEffortLimits_);
//     // paramInterface->loadParameter("enforce_velocity_limits",       enforceVelocityLimits_);
//     // paramInterface->loadParameter("enforce_lower_position_limits", enforceLowerPositionLimits_);
//     // paramInterface->loadParameter("enforce_upper_position_limits", enforceUpperPositionLimits_);
//     return true;
// }

bool ControlItParameters::loadReflectedRotorInertias()
{
    Vector rotorInertias;
    if (paramInterface->loadParameter("controlit/rotor_inertias", rotorInertias))
    {
        Vector gearRatios;
        if (paramInterface->loadParameter("controlit/gear_ratios", gearRatios))
        {
            // Quick sanity check
            if (rotorInertias.size() == gearRatios.size())
            {
                #define NUM_VIRTUAL_DOFS 6

                reflectedRotorInertias.setZero(rotorInertias.size() + NUM_VIRTUAL_DOFS);

                for (int ii = 0; ii < rotorInertias.size(); ii++)
                {
                    reflectedRotorInertias[ii + NUM_VIRTUAL_DOFS] = rotorInertias[ii] * pow(gearRatios[ii], 2);
                }
                hasReflectedRotorInertias_ = true;
            }
            else
            {
                CONTROLIT_ERROR << "Rotor inertia vector and gear ratio vector do not have the same length!\n"
                                   "  - rotor inertias: " << rotorInertias.transpose() << "\n"
                                   "  - gear ratios: " << gearRatios.transpose();
                return false;
            }
        }
    }
    return true;
}

bool ControlItParameters::getParametersServiceHandler(controlit_core::getControlItParameters::Request & req,
            controlit_core::getControlItParameters::Response &res)
{
   return saveParameters(res.controlItParameters);
}

bool ControlItParameters::updateParametersServiceHandler(controlit_core::updateControlItParameters::Request & req,
            controlit_core::updateControlItParameters::Response &res)
{
    ros::NodeHandle nh;
    if (loadParameters(nh))
        return saveParameters(res.controlItParameters);
    else
    {
        res.controlItParameters.level = diagnostic_msgs::DiagnosticStatus::ERROR;
        res.controlItParameters.name = "WBC Parameters";
        res.controlItParameters.message = "Unable to reload parameters";
        res.controlItParameters.hardware_id = "n/a";
        return true;
    }
}

bool ControlItParameters::saveParameters(diagnostic_msgs::DiagnosticStatus & statusMsg)
{
    statusMsg.level = diagnostic_msgs::DiagnosticStatus::OK;
    statusMsg.name = "WBC Parameters";
    statusMsg.message = "These are the WBC parameters";
    statusMsg.hardware_id = "n/a";

    diagnostic_msgs::KeyValue kv;

    kv.key = "servo clock type";
    kv.value = servoClockType;
    statusMsg.values.push_back(kv);

    kv.key = "servo frequency";
    kv.value = static_cast<std::ostringstream*>(&(std::ostringstream() << servoFrequency))->str();
    statusMsg.values.push_back(kv);

    kv.key = "robot interface type";
    kv.value = robotInterfaceType;
    statusMsg.values.push_back(kv);

    kv.key = "controller type";
    kv.value = controllerType;
    statusMsg.values.push_back(kv);

    kv.key = "control model threading type";
    kv.value = useSingleThreadedControlModel_ ? "single-threaded" : "multi-threaded";
    statusMsg.values.push_back(kv);

    kv.key = "task updater threading type";
    kv.value = useSingleThreadedTaskUpdater_ ? "single-threaded" : "multi-threaded";
    statusMsg.values.push_back(kv);

    // kv.key = "sensor updater threading type";
    // kv.value = useSingleThreadedSensorUpdater_ ? "single-threaded" : "multi-threaded";
    // statusMsg.values.push_back(kv);

    // kv.key = "effort limits";
    // if (hasTorqueLimits_)
    //     kv.value = static_cast<std::ostringstream*>(&(std::ostringstream() << torqueLimits.transpose()))->str();
    // else
    //     kv.value = "none";
    // statusMsg.values.push_back(kv);

    // kv.key = "velocity limits";
    // if (hasVelocityLimits_)
    //     kv.value = static_cast<std::ostringstream*>(&(std::ostringstream() << velocityLimits.transpose()))->str();
    // else
    //     kv.value = "none";
    // statusMsg.values.push_back(kv);

    // kv.key = "position lower limits";
    // if (hasPositionLowerLimits_)
    //     kv.value = static_cast<std::ostringstream*>(&(std::ostringstream() << positionLowerLimits.transpose()))->str();
    // else
    //     kv.value = "none";
    // statusMsg.values.push_back(kv);

    // kv.key = "position upper limits";
    // if (hasPositionUpperLimits_)
    //     kv.value = static_cast<std::ostringstream*>(&(std::ostringstream() << positionUpperLimits.transpose()))->str();
    // else
    //     kv.value = "none";
    // statusMsg.values.push_back(kv);

    // kv.key = "enforce effort limits";
    // kv.value = enforceEffortLimits_ ? "True" : "False";
    // statusMsg.values.push_back(kv);

    // kv.key = "enforce velocity limits";
    // kv.value = enforceVelocityLimits_ ? "True" : "False";
    // statusMsg.values.push_back(kv);

    // kv.key = "enforce lower position limits";
    // kv.value = enforceLowerPositionLimits_ ? "True" : "False";
    // statusMsg.values.push_back(kv);

    // kv.key = "enforce upper position limits";
    // kv.value = enforceUpperPositionLimits_ ? "True" : "False";
    // statusMsg.values.push_back(kv);

    // kv.key = "torque offsets";
    // if (hasTorqueOffsets_)
    //     kv.value = static_cast<std::ostringstream*>(&(std::ostringstream() << torqueOffsets.transpose()))->str();
    // else
    //     kv.value = "none";
    // statusMsg.values.push_back(kv);

    // kv.key = "torque scaling factors";
    // if (hasTorqueScalingFactors_)
    //     kv.value = static_cast<std::ostringstream*>(&(std::ostringstream() << torqueScalingFactors.transpose()))->str();
    // else
    //     kv.value = "none";
    // statusMsg.values.push_back(kv);

    // kv.key = "ramp up time";
    // if (hasRampUpTime_)
    //     kv.value = static_cast<std::ostringstream*>(&(std::ostringstream() << rampUpTime))->str();
    // else
    //     kv.value = "none";
    // statusMsg.values.push_back(kv);

    kv.key = "coupled joint groups";
    if (hasCoupledJointGroups_)
    {
        std::stringstream msgBuff;
        msgBuff << "[";
        for (size_t ii = 0; ii < coupledJointGroups->size(); ii++)
        {
            msgBuff << "[";
            for (size_t jj = 0; jj < (*coupledJointGroups)[ii].size(); jj++)
            {
                msgBuff << (*coupledJointGroups)[ii][jj];
                if (jj < (*coupledJointGroups)[ii].size() - 1)
                    msgBuff << ", ";
            }
            msgBuff << "]";
            if (ii < coupledJointGroups->size() - 1)
                msgBuff << ", ";
        }
        msgBuff << "]";
        kv.value = msgBuff.str();
    }
    else
        kv.value = "none";
    statusMsg.values.push_back(kv);

    kv.key = "gravity";
    kv.value = static_cast<std::ostringstream*>(&(std::ostringstream() << gravityVector.transpose()))->str();
    statusMsg.values.push_back(kv);

    kv.key = "gravity compensation mask";
    if (hasGravCompMask_)
    {
        std::stringstream msgBuff;
        msgBuff << "[";
        for (size_t ii = 0; ii < gravCompMask->size(); ii++)
        {
            msgBuff << (*gravCompMask)[ii];
            if (ii < gravCompMask->size() - 1)
                msgBuff << ", ";
        }
        msgBuff << "]";
        kv.value = msgBuff.str();
    }
    else
        kv.value = "none";
    statusMsg.values.push_back(kv);

    // kv.key = "effective gain pass through mask";
    // if (hasEffectiveGainPassthroughMask_)
    // {
    //     std::stringstream msgBuff;
    //     msgBuff << "[";
    //     for (size_t ii = 0; ii < effectiveGainPassThroughMask->size(); ii++)
    //     {
    //         msgBuff << (*effectiveGainPassThroughMask)[ii];
    //         if (ii < effectiveGainPassThroughMask->size() - 1)
    //             msgBuff << ", ";
    //     }
    //     msgBuff << "]";
    //     kv.value = msgBuff.str();
    // }
    // else
    //     kv.value = "none";
    // statusMsg.values.push_back(kv);

    return true;
}

// void ControlItParameters::resizeLimits(int size)
// {
//     torqueLimits.setZero(size);
//     velocityLimits.setZero(size);
//     positionLowerLimits.setZero(size);
//     positionUpperLimits.setZero(size);
// }

// void ControlItParameters::saveLimits(WBCJointLimits &limits)
// {
//     torqueLimits = limits.torqueLimits;
//     velocityLimits = limits.velocityLimits;
//     positionLowerLimits = limits.positionLowerLimits;
//     positionUpperLimits = limits.positionUpperLimits;

//     hasTorqueLimits_ = true;
//     hasVelocityLimits_ = true;
//     hasPositionLowerLimits_ = true;
//     hasPositionUpperLimits_ = true;
// }

// std::string ControlItParameters::controllerTypeToString(ControllerType controllerType)
// {
//     switch(controllerType)
//     {
//         case ControllerType::IMPEDANCE: return "WBOSC_Impedance";
//         case ControllerType::TORQUE: return "WBOSC";
//         default: return "Unknown";
//     }
// }

} // namespace utility
} // namespace controlit