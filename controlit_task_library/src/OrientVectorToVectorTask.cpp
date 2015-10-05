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

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_mathutils.h>
#include <controlit/task_library/OrientVectorToVectorTask.hpp>

#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

namespace controlit {
namespace task_library {

// Uncomment one of the following lines to enable/disable detailed debug statements.
#define PRINT_DEBUG_STATEMENT(ss)
// #define PRINT_DEBUG_STATEMENT(ss) CONTROLIT_PR_DEBUG << ss;

#define PRINT_DEBUG_STATEMENT_RT(ss)
// #define PRINT_DEBUG_STATEMENT_RT(ss) CONTROLIT_PR_DEBUG_RT << ss;


OrientVectorToVectorTask::OrientVectorToVectorTask() :
    LatchedTask("__UNNAMED_OrientVectorToVectorTask_TASK__", CommandType::ACCELERATION, new TaskState(), new TaskState()),
    bodyName_(""),
    bodyId_(std::numeric_limits<unsigned int>::max()),
    markerTopic("visualizationMarkers")
{
    declareParameter("bodyFrameVector", & bodyFrameVector_);
    declareParameter("goalVector",      & goalVector_);
    declareParameter("bodyName",        & bodyName_);
    declareParameter("markerTopic",     & markerTopic);

    paramErrorAngle = declareParameter("errorAngle",       & errorAngle);
    paramGoalHeading = declareParameter("goalHeading",     & goalHeading);
    paramActualHeading = declareParameter("actualHeading", & actualHeading);

    // Create the PD controller
    controller.reset(PDControllerFactory::create(SaturationPolicy::NormVel));

    // Add controller parameters to this task
    controller->declareParameters(this);
}

void OrientVectorToVectorTask::addDefaultBindings()
{
    if (!useDefaultBindings) return;
    if (!hasBinding("bodyFrameVector")) addParameter(createROSInputBinding("bodyFrameVector", "std_msgs/Float64MultiArray"));
    if (!hasBinding("goalVector"))      addParameter(createROSInputBinding("goalVector", "std_msgs/Float64MultiArray"));
    if (!hasBinding("bodyName"))        addParameter(createROSInputBinding("bodyName", "std_msgs/String"));
    if (!hasBinding("markerTopic"))     addParameter(createROSInputBinding("markerTopic", "std_msgs/String"));
    if (!hasBinding("kp"))              addParameter(createROSInputBinding("kp", "std_msgs/Float64"));
    if (!hasBinding("kd"))              addParameter(createROSInputBinding("kd", "std_msgs/Float64"));
    if (!hasBinding("enableState"))     addParameter(createROSInputBinding("enableState", "std_msgs/Int32"));
    if (!hasBinding("tare"))            addParameter(createROSInputBinding("tare", "std_msgs/Int32"));

    if (!hasBinding("error"))               addParameter(createROSOutputBinding("error", "std_msgs/Float64MultiArray"));
    if (!hasBinding("errorDot"))            addParameter(createROSOutputBinding("errorDot", "std_msgs/Float64MultiArray"));
    if (!hasBinding("errorNorm"))           addParameter(createROSOutputBinding("errorNorm", "std_msgs/Float64"));
    if (!hasBinding("errorDotNorm"))        addParameter(createROSOutputBinding("errorDotNorm", "std_msgs/Float64"));
    if (!hasBinding("errorAngle"))          addParameter(createROSOutputBinding("errorAngle", "std_msgs/Float64"));
    if (!hasBinding("actualHeading"))       addParameter(createROSOutputBinding("actualHeading", "std_msgs/Float64MultiArray"));
    if (!hasBinding("goalHeading"))         addParameter(createROSOutputBinding("goalHeading", "std_msgs/Float64MultiArray"));
    if (!hasBinding("PDCommand"))           addParameter(createROSOutputBinding("PDCommand", "std_msgs/Float64MultiArray"));
}

bool OrientVectorToVectorTask::init(ControlModel & model)
{
    PRINT_DEBUG_STATEMENT("Method called!")

    // Initialize the parent class (frameId lookup)

    // Abort if parent class fails to initialize
    if (!LatchedTask::init(model)) return false;
    if (!model.getBodyID(bodyName_, bodyId_)) return false;

    addDefaultBindings();

    // Resize the controller.
    controller->resize(3);
    e0.resize(3);
    JwBody.resize(3, model.getNumDOFs());
    JwFrame.resize(3, model.getNumDOFs());
    Jtloc.resize(3, model.getNumDOFs());
    Rbody.resize(3, 3);
    Rframe.resize(3, 3);

    // CONTROLIT_PR_INFO << "bodyId_ = " << bodyId_ << ", frameId_ = " << frameId_;

    if (bodyFrameVector_.rows() != 3)
    {
        CONTROLIT_PR_ERROR << "bodyFrameVector_ must have 3 dimensions, got " << bodyFrameVector_.rows();
        return false;
    }

    if (goalVector_.rows() != 3)
    {
        CONTROLIT_PR_ERROR << "goalVector_ must have 3 dimensions, got " << goalVector_.rows();
        return false;
    }

    // Normalize the current heading and goal heading vectors
    bodyFrameVector_.normalize();
    goalVector_.normalize();

    // Create a real-time-safe ROS topic publisher for visualizing the current and goal
    // heading vectors
    visualizationPublisher.init(getInstanceName() + "/" + markerTopic, 1);


    // Initialize the message within the visualization publisher
    while (!visualizationPublisher.trylock()) usleep(200);

    geometry_msgs::Point startPoint;
    geometry_msgs::Point endPoint;

    startPoint.x = startPoint.y = startPoint.z = 0;
    endPoint.x   = endPoint.y   = endPoint.z   = 0;

    // Create a marker showing the goal vector
    visualization_msgs::Marker goalMarker;
    goalMarker.header.frame_id = frameName_;
    goalMarker.header.stamp = ros::Time::now();
    goalMarker.ns = getTypeName() + "/" + getInstanceName();
    goalMarker.id = 0;
    goalMarker.type = visualization_msgs::Marker::ARROW;
    goalMarker.action = visualization_msgs::Marker::ADD;
    goalMarker.points.push_back(startPoint);
    goalMarker.points.push_back(endPoint);
    goalMarker.scale.x = 0.02; // arrow shaft diameter
    goalMarker.scale.y = 0.1;  // arrow head diamater
    goalMarker.scale.z = 0.0;
    goalMarker.color.a = 1.0;
    goalMarker.color.r = 0.0;
    goalMarker.color.g = 1.0;
    goalMarker.color.b = 0.0;

    // Create a marker showing the current heading
    visualization_msgs::Marker currMarker;
    currMarker.header.frame_id = bodyName_;
    currMarker.header.stamp = ros::Time::now();
    currMarker.ns = getTypeName() + "/" + getInstanceName();
    currMarker.id = 1;
    currMarker.type = visualization_msgs::Marker::ARROW;
    currMarker.action = visualization_msgs::Marker::ADD;
    currMarker.points.push_back(startPoint);
    currMarker.points.push_back(endPoint);
    currMarker.scale.x = 0.02; // arrow shaft diameter
    currMarker.scale.y = 0.08; // arrow head diamater
    currMarker.scale.z = 0.0;
    currMarker.color.a = 1.0;
    currMarker.color.r = 0.0;
    currMarker.color.g = 0.0;
    currMarker.color.b = 1.0;

    // Save the markers in a MarkerArray message
    visualizationPublisher.msg_.markers.clear();
    visualizationPublisher.msg_.markers.push_back(goalMarker);
    visualizationPublisher.msg_.markers.push_back(currMarker);

    // Publish the MarkerArray
    visualizationPublisher.unlockAndPublish();

    Q.setZero(model.getNumDOFs());
    Qd.setZero(model.getNumDOFs());

    base_vector.setZero(3);
    tran_BodyToBase.setZero(3);

    return Task::init(model);
}

bool OrientVectorToVectorTask::updateStateImpl(ControlModel * model, TaskState * taskState)
{
    PRINT_DEBUG_STATEMENT("Method called!")

    assert(model != nullptr);
    assert(taskState != nullptr);

    Matrix & taskJacobian = taskState->getJacobian();

    int numDOFs = model->getNumDOFs();

    // Normalize vectors in case they have been updated
    bodyFrameVector_.normalize();
    goalVector_.normalize();

    // Check if latched status has been updated
    updateLatch(model);

    if(taskJacobian.rows() != 3 || taskJacobian.cols() != numDOFs)
    {
        PRINT_DEBUG_STATEMENT("Resizing task Jacobian to be 3x" << numDOFs)
        taskJacobian.resize(3, numDOFs);
    }

    RigidBodyDynamics::CalcPointJacobianW(model->rbdlModel(), model->getQ(), bodyId_, Vector::Zero(3), JwBody, false);
    Rbody = RigidBodyDynamics::CalcBodyWorldOrientation(model->rbdlModel(), model->getQ(), bodyId_, false);

    Vector RTeBody = Rbody.transpose() * bodyFrameVector_;
    taskJacobian = -RigidBodyDynamics::Math::VectorCrossMatrix(RTeBody) * JwBody;

    if(frameId_ != -1) // Goal vector is in a robot reference frame, either latched or unlatched
    {
        if(!isLatched) // Need to account for relative motion of the reference frame in the Jacobian
        {
            RigidBodyDynamics::CalcPointJacobianW(model->rbdlModel(), model->getQ(), frameId_, Vector::Zero(3), JwFrame, false);
            Rframe = RigidBodyDynamics::CalcBodyWorldOrientation(model->rbdlModel(), model->getQ(), frameId_, false);
            Vector goal = Rframe.transpose() * goalVector_;
            taskJacobian += RigidBodyDynamics::Math::VectorCrossMatrix(goal) * JwFrame;
        }
    }

    return true;
}

bool OrientVectorToVectorTask::sense(ControlModel & model)
{
    // Get the latest joint state information
    model.getLatestFullState(Q, Qd);

    // Get orientation of the local body vector in the world frame
    Rbody = RigidBodyDynamics::CalcBodyWorldOrientation(model.rbdlModel(), Q, bodyId_, false);  // TODO: Use Latest State

    if (tare)
    {
        CONTROLIT_INFO_RT << "Taring the goal orientation!";
        goalVector_ = Rbody.transpose() * bodyFrameVector_;   // assumes goalVector_ is in world coordinate frame
        tare = 0;
    }

    // Just in case, normalize the current and goal vectors
    bodyFrameVector_.normalize();
    goalVector_.normalize();

    // Check if latched status has been updated
    updateLatch(&model);

    getJacobian(Jtloc);

    // Compute the goal vector in the frameName_ coordinate frame
    // goalHeading = goalVector_;
    paramGoalHeading->set(goalVector_); // sets member variable "goalHeading"

    if(frameId_ != -1) // The goal vector is in robot frame, either latched or not
    {
        if(isLatched)
            Rframe = latchedRotation;
        else
            Rframe = RigidBodyDynamics::CalcBodyWorldOrientation(model.rbdlModel(), Q, frameId_, false);  // TODO: Use Latest State

        goalHeading = Rframe.transpose() * goalVector_;
    }

    // Get the heading in the frameName_ coordinate frame
    actualHeading = Rbody.transpose() * bodyFrameVector_;
    paramActualHeading->set(actualHeading);

    return true;
}

bool OrientVectorToVectorTask::getCommand(ControlModel & model, TaskCommand & command)
{
    if (!sense(model)) return false;
    // Compute the error (goal heading - current heading)
    /*if (goalHeading.cols() != actualHeading.cols() || goalHeading.rows() != actualHeading.rows())
    {
        CONTROLIT_ERROR << "Matrix size error:\n"
                        << "  - goalHeading =\n" << goalHeading << "\n"
                        << "  - goalVector =\n" << goalVector_ << "\n"
                        << "  - frameId_ = " << frameId_ << "\n"
                        << "  - RFrame =\n" << Rframe << "\n"
                        << "  - actualHeading=\n" << actualHeading;
    }*/
    e0 = goalHeading - actualHeading;

    // Compute the angle between the goal heading and current heading.
    // The equation is cos^-1(goalHeading dot actualHeading / (|goalHeading| * |actualHeading|)).
    double error = std::acos(goalHeading.dot(actualHeading) / (goalHeading.norm() * actualHeading.norm()))
        * 180 / 3.141592653589793238463;
    paramErrorAngle->set(error);

    // Set the command type
    command.type = commandType_;

    // Compute the command
    // The goal velocity is zero, thus the velocity error is -Jtloc * Qd.
    controller->computeCommand(e0, -Jtloc * Qd, command.command, this);

    tran_BodyToBase = RigidBodyDynamics::CalcBodyToBaseCoordinates(model.rbdlModel(), Q, bodyId_, base_vector, false);

    // If we are able to grab the lock on visualizationPublisher,
    // publish the visualization markers.
    if (visualizationPublisher.trylock())
    {
        // Create a marker showing the goal vector
        visualizationPublisher.msg_.markers[0].header.stamp = ros::Time::now();
        visualizationPublisher.msg_.markers[0].points[0].x = tran_BodyToBase[0];
        visualizationPublisher.msg_.markers[0].points[0].y = tran_BodyToBase[1];
        visualizationPublisher.msg_.markers[0].points[0].z =tran_BodyToBase[2];

        visualizationPublisher.msg_.markers[0].points[1].x = tran_BodyToBase[0] + goalHeading(0);
        visualizationPublisher.msg_.markers[0].points[1].y = tran_BodyToBase[1] + goalHeading(1);
        visualizationPublisher.msg_.markers[0].points[1].z = tran_BodyToBase[2] + goalHeading(2);

        // Create a marker showing the current heading
        visualizationPublisher.msg_.markers[1].header.stamp = ros::Time::now();
        visualizationPublisher.msg_.markers[1].points[1].x = bodyFrameVector_(0);
        visualizationPublisher.msg_.markers[1].points[1].y = bodyFrameVector_(1);
        visualizationPublisher.msg_.markers[1].points[1].z = bodyFrameVector_(2);

        // Publish the MarkerArray
        visualizationPublisher.unlockAndPublish();
    }

    return true;
}

} // namespace task_library
} // namespace controlit
