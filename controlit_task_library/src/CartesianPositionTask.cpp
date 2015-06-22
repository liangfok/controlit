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

#include <limits>
#include <controlit/task_library/CartesianPositionTask.hpp>
#include <visualization_msgs/MarkerArray.h>

namespace controlit {
namespace task_library {

// Uncomment one of the following lines to enable/disable detailed debug statements.
#define PRINT_DEBUG_STATEMENT(ss)
// #define PRINT_DEBUG_STATEMENT(ss) CONTROLIT_PR_DEBUG << ss;

#define PRINT_DEBUG_STATEMENT_RT(ss)
// #define PRINT_DEBUG_STATEMENT_RT(ss) CONTROLIT_PR_DEBUG_RT << ss;

CartesianPositionTask::CartesianPositionTask() :
    LatchedTask("__UNNAMED_CARTPOS_TASK__", CommandType::ACCELERATION, new TaskState(), new TaskState()),
    bodyName_(""),
    bodyId_(std::numeric_limits<unsigned int>::max()),
    paramActualPosition(NULL),
    paramActualWorldPosition(NULL),
    paramActualVelocity(NULL),
    paramActualWorldVelocity(NULL)
{
    declareParameter("goalPosition", &goalPosition_);
    declareParameter("goalVelocity", &goalVelocity_);
    paramActualPosition = declareParameter("actualPosition", &actualPosition_);
    paramActualWorldPosition = declareParameter("actualWorldPosition", &actualWorldPosition_);
    paramActualVelocity = declareParameter("actualVelocity", &actualVelocity_);
    paramActualWorldVelocity = declareParameter("actualWorldVelocity", &actualWorldVelocity_);
    declareParameter("bodyName", &bodyName_);
    declareParameter("controlPoint", &controlPoint_);
    declareParameter("projection", &projection_);

    // Create the PD controller
    controller.reset(PDControllerFactory::create(SaturationPolicy::NormVel));

    // Add controller parameters to this task
    controller->declareParameters(this);
}

void CartesianPositionTask::addDefaultBindings()
{
    if (!useDefaultBindings) return;
    if (!hasBinding("goalPosition")) addParameter(createROSInputBinding("goalPosition", "std_msgs/Float64MultiArray"));
    if (!hasBinding("goalVelocity")) addParameter(createROSInputBinding("goalVelocity", "std_msgs/Float64MultiArray"));
    if (!hasBinding("kp"))           addParameter(createROSInputBinding("kp", "std_msgs/Float64"));
    if (!hasBinding("kd"))           addParameter(createROSInputBinding("kd", "std_msgs/Float64"));
    if (!hasBinding("enableState"))  addParameter(createROSInputBinding("enableState", "std_msgs/Int32"));
    if (!hasBinding("tare"))         addParameter(createROSInputBinding("tare", "std_msgs/Int32"));
    if (!hasBinding("bodyName"))     addParameter(createROSInputBinding("bodyName", "std_msgs/String"));
    if (!hasBinding("controlPoint")) addParameter(createROSInputBinding("controlPoint", "std_msgs/Float64MultiArray"));
    if (!hasBinding("projection"))   addParameter(createROSInputBinding("projection", "std_msgs/Float64MultiArray"));

    if (!hasBinding("error"))               addParameter(createROSOutputBinding("error", "std_msgs/Float64MultiArray"));
    if (!hasBinding("errorDot"))            addParameter(createROSOutputBinding("errorDot", "std_msgs/Float64MultiArray"));
    if (!hasBinding("errorNorm"))           addParameter(createROSOutputBinding("errorNorm", "std_msgs/Float64"));
    if (!hasBinding("errorDotNorm"))        addParameter(createROSOutputBinding("errorDotNorm", "std_msgs/Float64"));
    if (!hasBinding("actualPosition"))      addParameter(createROSOutputBinding("actualPosition", "std_msgs/Float64MultiArray"));
    if (!hasBinding("actualVelocity"))      addParameter(createROSOutputBinding("actualVelocity", "std_msgs/Float64MultiArray"));
    if (!hasBinding("actualWorldPosition")) addParameter(createROSOutputBinding("actualWorldPosition", "std_msgs/Float64MultiArray"));
    if (!hasBinding("actualWorldVelocity")) addParameter(createROSOutputBinding("actualWorldVelocity", "std_msgs/Float64MultiArray"));
    if (!hasBinding("PDCommand"))           addParameter(createROSOutputBinding("PDCommand", "std_msgs/Float64MultiArray"));
}


bool CartesianPositionTask::init(ControlModel & model)
{

    std::stringstream buff;
    buff << "Parameters:\n";
    for (auto const& parameter : lookupParameters(controlit::PARAMETER_TYPE_BINDING))
    {
        buff << "  - " << parameter->name() << ", " << parameter->getBindingConfig()->toString() << "\n";
    }
    CONTROLIT_INFO << buff.str();

    addDefaultBindings();

    // Initialize parent class (frameId lookup)

    // Abort if parent class fails to initialize
    if (!LatchedTask::init(model)) return false;

    // Get the body ID using its name
    if (!model.getBodyID(bodyName_, bodyId_)) return false;

    // Resize the controller... really hate these magic numbers.
    // Tasks should have a strong sense of space and space should know
    // its dimension.
    controller->resize(3);
    JvBody.setZero(3, model.getNumDOFs());
    JwBody.setZero(3, model.getNumDOFs());
    JvFrame.setZero(3, model.getNumDOFs());
    JwFrame.setZero(3, model.getNumDOFs());
    JtLoc.setZero(3, model.getNumDOFs());
    actualPosition_.resize(3);
    actualWorldPosition_.resize(3);
    actualVelocity_.resize(3);
    actualWorldVelocity_.resize(3);

    xCurWorld.resize(3);
    xCurFrameProjected.resize(3);
    xCurWorldProjected.resize(3);

    xDesWorldProjected.resize(3);

    Q.setZero(model.getNumDOFs());
    Qd.setZero(model.getNumDOFs());

    // CONTROLIT_PR_INFO << "bodyId_ = " << bodyId_ << ", frameId_ = " << frameId_;

    // Ensure the dimensions of various parameters are correct.
    if(projection_.rows() != 3 || projection_.cols() != 3)
    {
        if (getEnableState() == EnableState::SENSING)
            projection_.setIdentity(3, 3);
        else
        {
            CONTROLIT_ERROR << "Projection matrix must have size 3 x 3, got "<< projection_.rows() <<" x "<<projection_.cols();
            return false;
        }
    }

    if (controlPoint_.rows() != 3)
    {
        if (getEnableState() == EnableState::SENSING)
            controlPoint_.setZero(3);
        else
        {
            CONTROLIT_ERROR << "Control point must have 3 dimensions, got " << controlPoint_.rows();
            return false;
        }
    }

    if (goalPosition_.rows() != 3)
    {
        if (getEnableState() == EnableState::SENSING)
            goalPosition_.setZero(3);
        else
        {
            CONTROLIT_ERROR << "Goal position must have 3 dimensions, got " << goalPosition_.rows();
            return false;
        }
    }

    if (goalVelocity_.rows() != 3)
    {
        if (getEnableState() == EnableState::SENSING)
            goalVelocity_.setZero(3);
        else
        {
            CONTROLIT_ERROR << "Goal Velocity must have 3 dimensions, got " << goalVelocity_.rows();
            return false;
        }
    }

    return Task::init(model);
}

bool CartesianPositionTask::updateStateImpl(ControlModel * model, TaskState * taskState)
{
    PRINT_DEBUG_STATEMENT("Method called!")

    assert(model != nullptr);
    assert(taskState != nullptr);

    Matrix & taskJacobian = taskState->getJacobian();

    // Get the number of DOFs (both real and virtual)
    int numDOFs = model->getNumDOFs();

    if (taskJacobian.rows() != 3 || taskJacobian.cols() != numDOFs)
    {
        PRINT_DEBUG_STATEMENT("Resizing task Jacobian to be 3x" << numDOFs)
        taskJacobian.resize(3, numDOFs);
    }

    //Check if latched status has been updated
    updateLatch(model);

    // Compute the Jacobian matrix that converts from a point on the body of a robot to the joint space
    RigidBodyDynamics::CalcPointJacobian(model->rbdlModel(), model->getQ(), bodyId_, Vector::Zero(3), JvBody, false);
    RigidBodyDynamics::CalcPointJacobianW(model->rbdlModel(), model->getQ(), bodyId_, Vector::Zero(3), JwBody, false);
    Vector3d bodyTranslation = RigidBodyDynamics::CalcBodyToBaseCoordinates(model->rbdlModel(), model->getQ(), bodyId_, Vector::Zero(3), false);
    Matrix3d bodyRotation =  RigidBodyDynamics::CalcBodyWorldOrientation(model->rbdlModel(), model->getQ(), bodyId_, false);

    // The goal is specified relative to the fixed world frame
    if(frameId_ == -1)
    {
        taskJacobian = projection_ * (-RigidBodyDynamics::Math::VectorCrossMatrix(bodyRotation.transpose() * controlPoint_) * JwBody + JvBody);
    }
    else // The goal is specified relative to a robot frame, either latched or not
    {
        Matrix3d frameRotation;
        Vector3d frameTranslation;

        if(isLatched)
        {
            frameRotation = latchedRotation;
            frameTranslation = latchedTranslation;
            taskJacobian = frameRotation.transpose() * projection_ * frameRotation * (-RigidBodyDynamics::Math::VectorCrossMatrix(bodyRotation.transpose() * controlPoint_) * JwBody + JvBody);
        }
        else
        {
            frameRotation =  RigidBodyDynamics::CalcBodyWorldOrientation(model->rbdlModel(), model->getQ(), frameId_, false);
            frameTranslation = RigidBodyDynamics::CalcBodyToBaseCoordinates(model->rbdlModel(), model->getQ(), frameId_, Vector::Zero(3), false);

            RigidBodyDynamics::CalcPointJacobian(model->rbdlModel(), model->getQ(), frameId_, Vector::Zero(3), JvFrame, false);
            RigidBodyDynamics::CalcPointJacobianW(model->rbdlModel(), model->getQ(), frameId_, Vector::Zero(3), JwFrame, false);

            taskJacobian = frameRotation.transpose() * projection_ * frameRotation * (-RigidBodyDynamics::Math::VectorCrossMatrix(bodyRotation.transpose() * controlPoint_) * JwBody + JvBody - JvFrame);
            Vector3d beta = bodyRotation.transpose() * controlPoint_ + bodyTranslation - frameTranslation;
            Vector3d alpha = frameRotation.transpose() * projection_ * frameRotation * beta;
            taskJacobian += (-RigidBodyDynamics::Math::VectorCrossMatrix(alpha)
                   + frameRotation.transpose() * projection_ * frameRotation * RigidBodyDynamics::Math::VectorCrossMatrix(beta)
                   + RigidBodyDynamics::Math::VectorCrossMatrix(frameRotation.transpose() * projection_ * goalPosition_)) * JwFrame;
            taskJacobian -= JvFrame;
        }
    }
    return true;
}

bool CartesianPositionTask::sense(ControlModel & model)
{
    // Get the latest joint state information
    model.getLatestFullState(Q, Qd);

    bodyTranslation = RigidBodyDynamics::CalcBodyToBaseCoordinates(model.rbdlModel(), Q, bodyId_, Vector::Zero(3), false);
    bodyRotation =  RigidBodyDynamics::CalcBodyWorldOrientation(model.rbdlModel(), Q, bodyId_, false);

    getJacobian(JtLoc);

    xCurWorld = bodyRotation.transpose() * controlPoint_ + bodyTranslation;


    if(frameId_ == -1) //Using fixed world frame as reference, latching immaterial.
    {

        // if (projection_.cols() != xCurWorld.rows())
        // {
        //     CONTROLIT_ERROR << "Matrix size problem:\n"
        //                     << "  - frameId_ = " << frameId_ << "\n"
        //                     << "  - projection_ = " << projection_ << "\n"
        //                     << "  - xCurWorld_ = " << xCurWorld;
        // }

        xCurFrameProjected = projection_ * xCurWorld;
        xCurWorldProjected = xCurFrameProjected;

        xDesWorldProjected = projection_ * goalPosition_;
        frameRotation.setIdentity();
    }
    else
    {
        if(!isLatched)
        {
            frameRotation =  RigidBodyDynamics::CalcBodyWorldOrientation(model.rbdlModel(), Q, frameId_, false);
            frameTranslation = RigidBodyDynamics::CalcBodyToBaseCoordinates(model.rbdlModel(), Q, frameId_, Vector::Zero(3), false);
        }
        else
        {
            frameRotation = latchedRotation;
            frameTranslation = latchedTranslation;
        }

        xCurFrameProjected = projection_ * frameRotation * (xCurWorld - frameTranslation);
        xCurWorldProjected = frameRotation.transpose() * xCurFrameProjected + frameTranslation;
        xDesWorldProjected = frameRotation.transpose() * projection_ * goalPosition_ + frameTranslation;
    }
    // publish parameter "actualPosition"
    paramActualPosition->set(xCurFrameProjected);
    paramActualWorldPosition->set(xCurWorldProjected);

    // std::cout<<"Virtual DOFs in CartesianPositionTask : " << Q.head(6).transpose()<<std::endl;
    ePos = xDesWorldProjected - xCurWorldProjected;
    //Debugging--only difference between compwise SMC controller and this is
    // the error_dot calculation...have now switch back -JtLoc * Qd to
    // see if it works since we're still having strange behevior w/ velocity
    // update fixed.
    eVel = -JtLoc * Qd;

    if(goalVelocity_.norm() > 0)
    {
        eVel += frameRotation.transpose() * projection_ * goalVelocity_;
    }

    return true;
}

bool CartesianPositionTask::getCommand(ControlModel & model, TaskCommand & command)
{
    PRINT_DEBUG_STATEMENT_RT("Method called!")

    if (tare)
    {
        CONTROLIT_INFO_RT << "Taring the goal position!";
        goalPosition_ = RigidBodyDynamics::CalcBodyToBaseCoordinates(model.rbdlModel(), model.getQ(),
            bodyId_, controlPoint_, true);
        tare = 0;
    }

    if (!sense(model)) return false;

    // Set the command type
    command.type = commandType_;

    // Compute the command
    controller->computeCommand(ePos,  eVel, command.command, this);
    // controller->computeCommand(ePos,  -JtLoc * Qd, command.command, this);

    PRINT_DEBUG_STATEMENT_RT("Done method call.\n"
                             " - command = " << command.command.transpose());
    return true;
}

} // namespace task_library
} // namespace controlit
