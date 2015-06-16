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

#include <controlit/task_library/JointPositionTask.hpp>

#include <controlit/utility/string_utility.hpp>
#include <controlit/addons/eigen/LinearAlgebra.hpp>

namespace controlit {
namespace task_library {

// Uncomment one of the following lines to enable/disable detailed debug statements.
#define PRINT_DEBUG_STATEMENT(ss)
// #define PRINT_DEBUG_STATEMENT(ss) CONTROLIT_PR_DEBUG << ss;

#define PRINT_DEBUG_STATEMENT_RT(ss)
// #define PRINT_DEBUG_STATEMENT_RT(ss) CONTROLIT_PR_DEBUG_RT << ss;

// #define PRINT_DEBUG_STATEMENT_RT_ALWAYS(ss) CONTROLIT_DEBUG_RT << ss;
#define PRINT_DEBUG_STATEMENT_RT_ALWAYS(ss) std::cout << ss << std::endl;

JointPositionTask::JointPositionTask() :
  controlit::Task("__UNNAMED_JPOS_TASK__", CommandType::ACCELERATION, new TaskState(), new TaskState()),
  paramActualPos(NULL),
  paramActualVel(NULL),
  paramCurrentGoal(NULL)
{
    declareParameter("goalAcceleration", &goalAcceleration);
    declareParameter("goalPosition", &goalPosition);
    declareParameter("goalVelocity", &goalVelocity);
    paramActualPos = declareParameter("actualPosition", &actualPosition);
    paramCurrentGoal = declareParameter("currentGoalPosition", &currentGoalPosition);
    paramActualVel = declareParameter("actualVelocity", &actualVelocity);
    paramCurrentGoalAccel = declareParameter("currentGoalAcceleration", &currentGoalAcceleration);
  
    // Create the PD controller
    controller.reset(PDControllerFactory::create(SaturationPolicy::ComponentWiseVel));
  
    // Add controller parameters to this task
    controller->declareParameters(this);

    // ============================================================================
    // BEGIN DREAMER SPECIFIC CODE!
    // jointMask[0] = true; // torso_lower_pitch
    // jointMask[1] = true; // torso_upper_pitch
    // jointMask[2] = true; // left_shoulder_extensor
    // jointMask[3] = true; // left_shoulder_abductor
    // jointMask[4] = true; // left_shoulder_rotator
    // jointMask[5] = true; // left_elbow
    // jointMask[6] = true; // left_wrist_rotator
    // jointMask[7] = true; // left_wrist_pitch
    // jointMask[8] = true; // left_wrist_yaw
    // jointMask[9] = true; // right_shoulder_extensor
    // jointMask[10] = true; // right_shoulder_abductor
    // jointMask[11] = true; // right_shoulder_rotator
    // jointMask[12] = true; // right_elbow
    // jointMask[13] = true; // right_wrist_rotator
    // jointMask[14] = true; // right_wrist_pitch
    // jointMask[15] = true; // right_wrist_yaw

    // numUnmaskedJoints = 0;
    // for (size_t ii = 0; ii < 16; ii++)
    // {
    //     if(jointMask[ii])
    //         numUnmaskedJoints++;
    // }
    // maskedJacobian.resize(numUnmaskedJoints, 22);
    // fullCommand.resize(16);
    // END DREAMER-SPECIFIC CODE!
    // ============================================================================
}

void JointPositionTask::addDefaultBindings()
{
    // CONTROLIT_INFO << "Method Called!\n"
    //     << "  - useDefaultBindings = " << useDefaultBindings;
    if (!useDefaultBindings) return;
    if (!hasBinding("goalPosition"))     addParameter(createROSInputBinding("goalPosition", "std_msgs/Float64MultiArray"));
    if (!hasBinding("goalVelocity"))     addParameter(createROSInputBinding("goalVelocity", "std_msgs/Float64MultiArray"));
    if (!hasBinding("goalAcceleration")) addParameter(createROSInputBinding("goalAcceleration", "std_msgs/Float64MultiArray"));
    if (!hasBinding("kp"))               addParameter(createROSInputBinding("kp", "std_msgs/Float64MultiArray"));
    if (!hasBinding("kd"))               addParameter(createROSInputBinding("kd", "std_msgs/Float64MultiArray"));
    if (!hasBinding("enableState"))      addParameter(createROSInputBinding("enableState", "std_msgs/Int32"));
    if (!hasBinding("tare"))             addParameter(createROSInputBinding("tare", "std_msgs/Int32"));

    if (!hasBinding("error"))                   addParameter(createROSOutputBinding("error", "std_msgs/Float64MultiArray"));
    if (!hasBinding("errorDot"))                addParameter(createROSOutputBinding("errorDot", "std_msgs/Float64MultiArray"));
    if (!hasBinding("errorNorm"))               addParameter(createROSOutputBinding("errorNorm", "std_msgs/Float64"));
    if (!hasBinding("errorDotNorm"))            addParameter(createROSOutputBinding("errorDotNorm", "std_msgs/Float64"));
    if (!hasBinding("actualPosition"))          addParameter(createROSOutputBinding("actualPosition", "std_msgs/Float64MultiArray"));
    if (!hasBinding("actualVelocity"))          addParameter(createROSOutputBinding("actualVelocity", "std_msgs/Float64MultiArray"));
    if (!hasBinding("currentGoalAcceleration")) addParameter(createROSOutputBinding("currentGoalAcceleration", "std_msgs/Float64MultiArray"));
    if (!hasBinding("PDCommand"))               addParameter(createROSOutputBinding("PDCommand", "std_msgs/Float64MultiArray"));
}

bool JointPositionTask::init(ControlModel & model)
{
  
    int realDofs = model.getNumRealDOFs();
    if (!controller->resize(realDofs))
    {
        CONTROLIT_ERROR << "Problems resizing PDController!";
        return false;
    }
  
    if (goalPosition.rows() != realDofs)
    {
        CONTROLIT_ERROR << "Goal position must have "
            << realDofs << " dimensions, got " << goalPosition.rows() << ".  This is because "
               " there are " << realDofs << " actuable DOFs in the model!";
        return false;
    }
  
    if (goalVelocity.rows() != realDofs)
    {
        CONTROLIT_ERROR << "Goal velocity must have "
            << realDofs << " dimensions, got " << goalVelocity.rows() << ".  This is because "
               " there are " << realDofs << " actuable DOFs in the model!";
        return false;
    }
  
    if (goalAcceleration.rows() != realDofs)
    {
        CONTROLIT_ERROR << "Goal acceleration must have "
            << realDofs << " dimensions, got " << goalAcceleration.rows() << ".  This is because "
               " there are " << realDofs << " actuable DOFs in the model!";
        return false;
    }
  
    // Allocate space for temporary variables
    errpos.resize(realDofs);
    errvel.resize(realDofs);
    currentGoalAcceleration.resize(realDofs); 

    addDefaultBindings();

    return Task::init(model);
}

bool JointPositionTask::updateStateImpl(ControlModel * model, TaskState * taskState)
{
    PRINT_DEBUG_STATEMENT("Method called!")
  
    assert(model != nullptr);
    assert(taskState != nullptr);
  
    // ============================================================================
    // BEGIN DREAMER SPECIFIC CODE!
    // Note: Hard coded for Dreamer. Remove the elbow joints from the Jacobian!
    // Matrix fullPostureJacobian(16,22);
    // model->constraints().getVirtualU(fullPostureJacobian);

    // size_t index = 0;
    // for (size_t ii = 0; ii < 16; ii++)
    // {
    //     if(jointMask[ii])
    //         maskedJacobian.block(index++, 0, 1, 22) = fullPostureJacobian.block(ii, 0, 1, 22);
    // }

    // taskState->getJacobian() = maskedJacobian;

    // CONTROLIT_INFO_RT << "Jacobian matrix:\n" << taskState->getJacobian();
    // END DREAMER-SPECIFIC CODE!
    // ============================================================================

    /*
     * This task's Jacobian is simply the underactuation matrix U
     * since it sets the joint state directly.
     *
     * The size of the Jacobian matrix is: number of actual DOFs x total number of DOFs
     * where the total number of DOFs includes the virtual DOFs.
     */
    model->constraints().getVirtualU(taskState->getJacobian());
  
    // PRINT_DEBUG_STATEMENT("Setting task's Jacobian to be the underactuation matrix:\n"
    //                      << taskState->getJacobian())
  
    PRINT_DEBUG_STATEMENT("Done.");
    return true;
}

// The getCommand method is inhereted from PDTask, which is the parent class
bool JointPositionTask::getCommand(ControlModel& model, TaskCommand & u)
{
    // CONTROLIT_INFO_RT 
    //     << "Method called!\n"
    //     << "  - tare = " << (tare == 1 ? "TRUE" : "FALSE");
  
  
    // #define TIME_J_POS_TASK_COMP_WISE_VEL_GET_COMMAND 1
  
    #ifdef TIME_J_POS_TASK_COMP_WISE_VEL_GET_COMMAND
    ros::Time startGetCurrentState = ros::Time::now();
    #endif
  
    // Obtain the current position and velocity of the joints.
    const Vector currPosition = model.getLatestJointState()->getJointPosition();
    const Vector currVelocity = model.getLatestJointState()->getJointVelocity();
  
    // Verify that the current position and velocity values are valid
    // if (!controlit::addons::eigen::checkMagnitude(currPosition) || !controlit::addons::eigen::checkMagnitude(currVelocity))
    // {
    //   CONTROLIT_ERROR_RT << "The current position or velocity contains invalid values:\n"
    //     << "  - currPosition: " << currPosition.transpose() << "\n"
    //     << "  - currVelocity: " << currVelocity.transpose();
    //   return false;
    // }

    if (tare)
    {
        CONTROLIT_INFO_RT << "Taring the goal position!";
        goalPosition = currPosition;
        tare = 0;
    }

    #ifdef TIME_J_POS_TASK_COMP_WISE_VEL_GET_COMMAND
    ros::Time startSizeCheck = ros::Time::now();
    #endif

    // Check to ensure the goal position vector is the right size.
    if (goalPosition.rows() != model.getNumRealDOFs())
    {
        std::stringstream errorMsgBuff;
        errorMsgBuff << "(instance = " << getInstanceName() << ", type = " << getTypeName() << "): "
                     << __func__ << ": Goal position has invalid length!\n"
                     << " - Expected " << model.getNumRealDOFs() << ", got " << goalPosition.rows();
        CONTROLIT_ERROR << errorMsgBuff.str();
        return false;
    }
  
    // Check to ensure the goal velocity vector is the right size.
    if (goalVelocity.rows() != model.getNumRealDOFs())
    {
        std::stringstream errorMsgBuff;
        errorMsgBuff << __func__ << ": Goal velocity has invalid length!\n"
                     << " - Expected " << model.getNumRealDOFs() << ", got " << goalVelocity.rows();
        CONTROLIT_ERROR << errorMsgBuff.str();
        return false;
    }
  
    #ifdef TIME_J_POS_TASK_COMP_WISE_VEL_GET_COMMAND
    ros::Time startComputeErrors = ros::Time::now();
    #endif
  
    // Compute the errors
    errpos = goalPosition - currPosition;
    errvel = goalVelocity - currVelocity;
  
    PRINT_DEBUG_STATEMENT_RT("Computed errors:\n"
                             "  - errpos = goalPosition - currPosition = [" << goalPosition.transpose() << "] - [" << currPosition.transpose() << "] = [" << errpos.transpose() << "]" << "\n"
                             "  - errvel = goalVelocity - currVelocity = [" << goalVelocity.transpose() << "] - [" << currVelocity.transpose() << "] = [" << errvel.transpose() << "]")
  
    // Verify that the error position and velocity values are valid
    if (!controlit::addons::eigen::checkMagnitude(errpos) || !controlit::addons::eigen::checkMagnitude(errpos))
    {
        CONTROLIT_ERROR_RT << "The position or velocity error contains invalid values:\n"
                           << "  - errpos: " << errpos.transpose() << "\n"
                           << "  - goalPosition: " << goalPosition.transpose() << "\n"
                           << "  - currPosition: " << currPosition.transpose() << "\n"
                           << "  - errvel: " << errvel.transpose() << "\n"
                           << "  - goalVelocity: " << goalVelocity.transpose() << "\n"
                           << "  - currVelocity: " << currVelocity.transpose();
        return false;
    }
  
    // Set the command type
    u.type = commandType_;
  
    #ifdef TIME_J_POS_TASK_COMP_WISE_VEL_GET_COMMAND
    ros::Time startComputeCommand = ros::Time::now();
    #endif
  
    // ============================================================================
    // BEGIN DREAMER SPECIFIC CODE!
    // Note: Hard coded for Dreamer. Remove the wrist joints from the Jacobian!
    // controller->computeCommand(errpos, errvel, fullCommand, this);
    // fullCommand += goalAcceleration;

    // u.command.resize(numUnmaskedJoints);

    // int index = 0;
    // for (int ii = 0; ii < 16; ii++)
    // {
    //     if (jointMask[ii])
    //         u.command[index++] = fullCommand[ii];
    // }

    // CONTROLIT_INFO_RT << "Command:\n"
    //                   << "  - Full: " << fullCommand.transpose() << "\n"
    //                   << "  - Masked: " << u.command.transpose();
    // END DREAMER-SPECIFIC CODE!
    // ============================================================================

    // Compute the command
    controller->computeCommand(errpos, errvel, u.command, this);
    u.command += goalAcceleration;
    
    #ifdef TIME_J_POS_TASK_COMP_WISE_VEL_GET_COMMAND
    ros::Time startUpdateParams = ros::Time::now();
    #endif
  
    paramActualPos->set(currPosition);   // sets local variable 'actualPosition'
    paramCurrentGoal->set(goalPosition); // sets local variable 'currentGoalPosition'
    paramActualVel->set(currVelocity);
    paramCurrentGoalAccel->set(goalAcceleration);
  
    #ifdef TIME_J_POS_TASK_COMP_WISE_VEL_GET_COMMAND
        ros::Time endUpdateParams = ros::Time::now();
        PRINT_DEBUG_STATEMENT_RT_ALWAYS(
            "JointPositionTask getCommand Latency Results (ms):\n"
            " - getCurrentState: " << (startSizeCheck - startGetCurrentState).toSec() * 1000 << "\n"
            " - sizeCheck: " << (startComputeErrors - startSizeCheck).toSec() * 1000 << "\n"
            " - computeErrors: " << (startComputeCommand - startComputeErrors).toSec() * 1000 << "\n"
            " - computeCommand: " << (startUpdateParams - startComputeCommand).toSec() * 1000 << "\n"
            " - updateParams: " << (endUpdateParams - startUpdateParams).toSec() * 1000);
    #endif
  
  
  
    // CONTROLIT_DEBUG_RT << "Method called.\n"
    //      " - goalPosition = " << goalPosition.transpose() << "\n"
    //      " - goalVelocity = " << goalVelocity.transpose() << "\n"
    //      " - currPosition = " << currPosition.transpose() << "\n"
    //      " - currVelocity = " << currVelocity.transpose() << "\n"
    //      " - errpos = " << errpos.transpose() << "\n"
    //      " - errvel = " << errvel.transpose() << "\n"
    //      " - kp = " << lookupParameter("kp")->getVector()->transpose() << "\n"
    //      " - kd = " << lookupParameter("kd")->getVector()->transpose() << "\n"
    //      " - u.command = " << u.command.transpose();
  
    // Print a nicely formatted command
  
    // PRINT_DEBUG_STATEMENT("Command:\n" << controlit::utility::prettyPrintJointSpaceCommand(model.getActuatedJointNamesVector(), u.command, "  "))
    // PRINT_DEBUG_STATEMENT("Done method call.")
  
    return true;
}

} // namespace task_library
} // namespace controlit

