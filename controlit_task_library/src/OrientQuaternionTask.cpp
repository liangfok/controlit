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
#include <controlit/task_library/OrientQuaternionTask.hpp>

namespace controlit {
namespace task_library {

// Uncomment one of the following lines to enable/disable detailed debug statements.
#define PRINT_DEBUG_STATEMENT(ss)
// #define PRINT_DEBUG_STATEMENT(ss) CONTROLIT_PR_DEBUG << ss;

#define PRINT_DEBUG_STATEMENT_RT(ss)
// #define PRINT_DEBUG_STATEMENT_RT(ss) CONTROLIT_PR_DEBUG_RT << ss;

OrientQuaternionTask::OrientQuaternionTask() :
    LatchedTask("__UNNAMED_ORIENTATION_TASK__", CommandType::ACCELERATION, new TaskState(), new TaskState()),
    bodyName_(""),
    bodyId_(std::numeric_limits<unsigned int>::max()),
    paramActualPosition(NULL),
    paramActualWorldPosition(NULL),
    markerTopic("visualizationMarkers"),
    markerLength(1),
    markerWidth(0.05),
    markerHeight(0.05)
{
    markerColorActual.setZero(4);
    markerColorActual << 1, 1, 0, 0;  // alpha  = 1, red = 1, green = 0, blue = 0

    markerColorGoal.setZero(4);
    markerColorGoal << 1, 0, 0, 1;    // alpha  = 1, red = 0, green = 0, blue = 1

    // The following is for testing purposes
    markerOrientation.setZero(4);
    markerOrientation << 0, 0, 0, 1;  // x, y, z, w

    declareParameter("bodyName",     & bodyName_);
    declareParameter("goalOrientation", & goalPosition_);
    declareParameter("goalVelocity", & goalVelocity_);
    declareParameter("controlPoint", & controlPoint_);

    declareParameter("markerTopic",       & markerTopic);
    declareParameter("markerLength",      & markerLength);
    declareParameter("markerWidth",       & markerWidth);
    declareParameter("markerHeight",      & markerHeight);
    declareParameter("markerColorGoal",   & markerColorGoal);
    declareParameter("markerColorActual", & markerColorActual);
    declareParameter("markerOrientation", & markerOrientation);  // for testing purposes

    paramActualPosition      = declareParameter("actualOrientation",      & actualPosition_);
    paramActualWorldPosition = declareParameter("actualWorldOrientation", & actualWorldPosition_);
    paramErrorQuat           = declareParameter("errorQuaternion",        & errorQuat_);

    // Create the PD controller
    controller.reset(PDControllerFactory::create(SaturationPolicy::NormVel));
  
    // Add controller parameters to this task
    controller->declareParameters(this);
}

void OrientQuaternionTask::addDefaultBindings()
{
    if (!useDefaultBindings) return;
    if (!hasBinding("goalOrientation"))   addParameter(createROSInputBinding("goalOrientation", "std_msgs/Float64MultiArray"));
    if (!hasBinding("goalVelocity"))      addParameter(createROSInputBinding("goalVelocity", "std_msgs/Float64MultiArray"));
    if (!hasBinding("controlPoint"))      addParameter(createROSInputBinding("controlPoint", "std_msgs/Float64MultiArray"));
    if (!hasBinding("bodyName"))          addParameter(createROSInputBinding("bodyName",     "std_msgs/String"));
    
    if (!hasBinding("markerTopic"))       addParameter(createROSInputBinding("markerTopic",       "std_msgs/String"));
    if (!hasBinding("markerLength"))      addParameter(createROSInputBinding("markerLength",      "std_msgs/Float64"));
    if (!hasBinding("markerWidth"))       addParameter(createROSInputBinding("markerWidth",       "std_msgs/Float64"));
    if (!hasBinding("markerHeight"))      addParameter(createROSInputBinding("markerHeight",      "std_msgs/Float64"));
    if (!hasBinding("markerColorGoal"))   addParameter(createROSInputBinding("markerColorGoal",   "std_msgs/Float64MultiArray"));
    if (!hasBinding("markerColorActual")) addParameter(createROSInputBinding("markerColorActual", "std_msgs/Float64MultiArray"));
    if (!hasBinding("markerOrientation")) addParameter(createROSInputBinding("markerOrientation", "std_msgs/Float64MultiArray"));
    
    if (!hasBinding("kp"))                addParameter(createROSInputBinding("kp",           "std_msgs/Float64"));
    if (!hasBinding("kd"))                addParameter(createROSInputBinding("kd",           "std_msgs/Float64"));
    if (!hasBinding("enableState"))       addParameter(createROSInputBinding("enableState",      "std_msgs/Int32"));
    if (!hasBinding("tare"))              addParameter(createROSInputBinding("tare",         "std_msgs/Int32"));
    
    if (!hasBinding("error"))               addParameter(createROSOutputBinding("error", "std_msgs/Float64MultiArray"));
    if (!hasBinding("errorDot"))            addParameter(createROSOutputBinding("errorDot", "std_msgs/Float64MultiArray"));
    if (!hasBinding("errorNorm"))           addParameter(createROSOutputBinding("errorNorm", "std_msgs/Float64"));
    if (!hasBinding("errorDotNorm"))        addParameter(createROSOutputBinding("errorDotNorm", "std_msgs/Float64"));
    if (!hasBinding("errorQuaternion"))        addParameter(createROSOutputBinding("errorQuaternion", "std_msgs/Float64MultiArray"));
    if (!hasBinding("actualOrientation"))      addParameter(createROSOutputBinding("actualOrientation", "std_msgs/Float64MultiArray"));
    // if (!hasBinding("actualVelocity"))      addParameter(createROSOutputBinding("actualVelocity", "std_msgs/Float64MultiArray"));
    if (!hasBinding("actualWorldOrientation")) addParameter(createROSOutputBinding("actualWorldOrientation", "std_msgs/Float64MultiArray"));
    // if (!hasBinding("actualWorldVelocity")) addParameter(createROSOutputBinding("actualWorldVelocity", "std_msgs/Float64MultiArray"));
    if (!hasBinding("PDCommand"))           addParameter(createROSOutputBinding("PDCommand", "std_msgs/Float64MultiArray"));
}

bool OrientQuaternionTask::init(ControlModel & model)
{
    PRINT_DEBUG_STATEMENT("Method called!")
  
    // Initialize the parent class (frameId lookup)
  
    // Abort if parent class fails to initialize
    if (!LatchedTask::init(model)) CONTROLIT_ERROR;

    addDefaultBindings();
  
    // Get the body ID using its name.
    if (!model.getBodyID(bodyName_, bodyId_)) return false;
  
    // Resize the controller and the matrices for this task
    controller->resize(9);
    e0.resize(9);
    e0dot.resize(9);
    Je1.resize(3, model.getNumDOFs());
    Je2.resize(3, model.getNumDOFs());
    Je3.resize(3, model.getNumDOFs());
    JwBody.resize(3, model.getNumDOFs());
    JwFrame.resize(3, model.getNumDOFs());
    JtLoc.resize(9, model.getNumDOFs());
  
    //resize optional output parameter
    actualPosition_.resize(4);
    actualWorldPosition_.resize(4);
    errorQuat_.resize(4);
  
    if (bodyName_.empty())
    {
       CONTROLIT_ERROR << "No body specified";
       return false;
    }
  
    if (goalPosition_.rows() != 4)
    {
        CONTROLIT_ERROR << "Goal position must have 4 (quaternion!) dimensions, got " << goalPosition_.rows();
        return false;
    }
  
    if (controlPoint_.rows() != 4)
    { 
        CONTROLIT_ERROR << "Control point must have 4 (quaternion!) dimensions, got " << controlPoint_.rows();
        return false;
    }
  
    if (goalVelocity_.rows() != 3)
    {
        CONTROLIT_ERROR << "Goal velocity must have 3 dimensions, got " << goalVelocity_.rows();
        return false;
    }
  
    // Create a real-time-safe ROS topic publisher for visualizing the current and goal
    // heading quaternions
    visualizationPublisher.init(getInstanceName() + "/" + markerTopic, 1);

    while (!visualizationPublisher.trylock()) usleep(200);

    visualization_msgs::Marker goalMarker;
    goalMarker.header.frame_id = "world";
    goalMarker.header.stamp = ros::Time::now();
    goalMarker.ns = getTypeName() + "/" + getInstanceName();
    goalMarker.id = 0;
    goalMarker.type = visualization_msgs::Marker::ARROW;
    goalMarker.action = visualization_msgs::Marker::ADD;

    base_vector.setZero(3);

    goalMarker.pose.position.x = 0;
    goalMarker.pose.position.y = 0;
    goalMarker.pose.position.z = 0;

    // This is for testing purposes
    goalMarker.pose.orientation.x = 0;
    goalMarker.pose.orientation.y = 0;
    goalMarker.pose.orientation.z = 0;
    goalMarker.pose.orientation.w = 0;

    goalMarker.scale.x = markerLength;
    goalMarker.scale.y = markerWidth;
    goalMarker.scale.z = markerHeight;

    goalMarker.color.a = markerColorGoal[0];

    goalMarker.color.r = markerColorGoal[1];
    goalMarker.color.g = markerColorGoal[2];
    goalMarker.color.b = markerColorGoal[3];

    // Create a marker showing the current heading
    // startPoint.x =0; startPoint.y=0;startPoint.z=0;
    // endPoint.x = bodyFrameVector_(0); endPoint.y = bodyFrameVector_(1); endPoint.z = bodyFrameVector_(2);
    
    // currMarker.header.frame_id = "world";
    // currMarker.header.stamp = ros::Time::now();
    // currMarker.ns = getTypeName() + "/" + getInstanceName();
    // currMarker.id = 1;
    // currMarker.type = visualization_msgs::Marker::ARROW;
    // currMarker.action = visualization_msgs::Marker::ADD;
    // currMarker.points.push_back(startPoint);
    // currMarker.points.push_back(endPoint);
    // currMarker.scale.x = 0.02; // arrow shaft diameter
    // currMarker.scale.y = 0.08; // arrow head diamater
    // currMarker.scale.z = 0.0;
    // currMarker.color.a = 1.0;
    // currMarker.color.r = 0.0;
    // currMarker.color.g = 0.0;
    // currMarker.color.b = 1.0;

    // Save the markers in a MarkerArray message
    visualizationPublisher.msg_.markers.clear();
    visualizationPublisher.msg_.markers.push_back(goalMarker);
    // visualizationPublisher.msg_.markers.push_back(currMarker);

    // Publish the MarkerArray
    visualizationPublisher.unlockAndPublish();

    return Task::init(model);
}

bool OrientQuaternionTask::updateStateImpl(ControlModel * model, TaskState * taskState)
{
    PRINT_DEBUG_STATEMENT("Method called!")
  
    assert(model != nullptr);
    assert(taskState != nullptr);
  
    // Obtain a reference to the task's Jacobian matrix.
    Matrix & taskJacobian = taskState->getJacobian();
  
    // Ensure the task's Jacobian matrix is the right size.
    if(taskJacobian.rows() != 9 || taskJacobian.cols() != (int)model->getNumDOFs())
        taskState->getJacobian().resize(9, model->getNumDOFs());
  
    // Check if latched status has been updated
    updateLatch(model);
  
    cpQuat.w() = controlPoint_[0];
    cpQuat.x() = controlPoint_[1];
    cpQuat.y() = controlPoint_[2];
    cpQuat.z() = controlPoint_[3];
    cpQuat.normalize();
  
    cpRot = cpQuat;
  
    // returns an orientation 3x3 matrix 
    bodyRot = RigidBodyDynamics::CalcBodyWorldOrientation(model->rbdlModel(), model->getQ(), bodyId_, false).transpose();

    // obtains the rotation Jacobian of the body(?)
    RigidBodyDynamics::CalcPointJacobianW(model->rbdlModel(), model->getQ(), bodyId_, Vector::Zero(3), JwBody, false);

    // Convert the control point quaternion given in local coordinate frame into global coordinate frame
    curRot = bodyRot * cpQuat.toRotationMatrix().transpose();
  
    // Convert the rotation Jacobian from local to global coordinate frame
    Je1 = -RigidBodyDynamics::Math::VectorCrossMatrix(curRot.col(0)) * JwBody;
    Je2 = -RigidBodyDynamics::Math::VectorCrossMatrix(curRot.col(1)) * JwBody;
    Je3 = -RigidBodyDynamics::Math::VectorCrossMatrix(curRot.col(2)) * JwBody;
  

    if(frameId_ != -1 && !isLatched) // Working in a frame which is moving with the robot
    {
        goalQuat.w() = goalPosition_[0];
        goalQuat.x() = goalPosition_[1];
        goalQuat.y() = goalPosition_[2];
        goalQuat.z() = goalPosition_[3];
    
        goalQuat.normalize();
        goalRot = goalQuat.toRotationMatrix();
    
        // This is 
        frameRot = RigidBodyDynamics::CalcBodyWorldOrientation(model->rbdlModel(), model->getQ(), frameId_, false);
        RigidBodyDynamics::CalcPointJacobianW(model->rbdlModel(), model->getQ(), frameId_, Vector::Zero(3), JwFrame, false);
        desRot = frameRot.transpose() * goalRot;
    
        Je1 += RigidBodyDynamics::Math::VectorCrossMatrix(desRot.col(0)) * JwFrame;
        Je2 += RigidBodyDynamics::Math::VectorCrossMatrix(desRot.col(1)) * JwFrame;
        Je3 += RigidBodyDynamics::Math::VectorCrossMatrix(desRot.col(2)) * JwFrame;
    }
  
    // Save the task Jacobian
    taskJacobian.block(0, 0, 3, model->getNumDOFs()) = Je1;
    taskJacobian.block(3, 0, 3, model->getNumDOFs()) = Je2;
    taskJacobian.block(6, 0, 3, model->getNumDOFs()) = Je3;
  
    return true;
}

bool OrientQuaternionTask::getCommand(ControlModel & model, TaskCommand & u)
{
    // Get the latest joint state information
    Vector Q(model.getNumDOFs());
    Vector Qd(model.getNumDOFs());
  
    model.getLatestFullState(Q, Qd);
  
    getJacobian(JtLoc);
  
    // TO-DO: Move this into the TaskState object!...ALREADY UPDATED!!!!
    bodyRot = RigidBodyDynamics::CalcBodyWorldOrientation(model.rbdlModel(), Q, bodyId_, false).transpose();
    bodyQuat = bodyRot;
    curRot = bodyRot * cpRot.transpose();
    ///////////////////////////////////////////
  
    Vector actualPositionTemp(4);
    Vector actualWorldPositionTemp(4);
    Vector errorQuatTemp(4);
  
    curQuat = curRot;
    actualWorldPositionTemp[0] = curQuat.w();
    actualWorldPositionTemp[1] = curQuat.x();
    actualWorldPositionTemp[2] = curQuat.y();
    actualWorldPositionTemp[3] = curQuat.z();
    actualPositionTemp = actualWorldPositionTemp;
  
    goalQuat.w() = goalPosition_[0];
    goalQuat.x() = goalPosition_[1];
    goalQuat.y() = goalPosition_[2];
    goalQuat.z() = goalPosition_[3];
    goalQuat.normalize();
    goalRot = goalQuat.toRotationMatrix();
  
    if(frameId_ != -1) // Working in a robot reference frame, either moving or not.
    {
        if(!isLatched)
            frameRot = RigidBodyDynamics::CalcBodyWorldOrientation(model.rbdlModel(), Q, frameId_, false);
        else
            frameRot = latchedRotation;
  
        curInFrameRot = frameRot * curRot;
        curInFrameQuat = curInFrameRot;
  
        actualPositionTemp[0] = curInFrameQuat.w();
        actualPositionTemp[1] = curInFrameQuat.x();
        actualPositionTemp[2] = curInFrameQuat.y();
        actualPositionTemp[3] = curInFrameQuat.z();
    }
    else
        frameRot.setIdentity();
  
    desRot = frameRot.transpose() * goalRot;
    desQuat = desRot;
  
    errQuat = desQuat.inverse() * curQuat;
    errorQuatTemp[0] = errQuat.w();
    errorQuatTemp[1] = errQuat.x();
    errorQuatTemp[2] = errQuat.y();
    errorQuatTemp[3] = errQuat.z();
  
    // Compute the orientation error
    e0.segment(0,3) = desRot.block(0,0,3,1) - curRot.block(0,0,3,1);
    e0.segment(3,3) = desRot.block(0,1,3,1) - curRot.block(0,1,3,1);
    e0.segment(6,3) = desRot.block(0,2,3,1) - curRot.block(0,2,3,1);
  
    // Set the command type
    u.type = commandType_;
  
    // Compute the orientation velocity error
    e0dot = -JtLoc * Qd;
    if(goalVelocity_.norm() > 0)
    {
        e0dot.segment(0,3) -= frameRot.transpose() * RigidBodyDynamics::Math::VectorCrossMatrix(goalRot.col(0)) * goalVelocity_;
        e0dot.segment(3,3) -= frameRot.transpose() * RigidBodyDynamics::Math::VectorCrossMatrix(goalRot.col(1)) * goalVelocity_;
        e0dot.segment(6,3) -= frameRot.transpose() * RigidBodyDynamics::Math::VectorCrossMatrix(goalRot.col(2)) * goalVelocity_;
    }
  
    // Compute the command
    controller->computeCommand(e0, e0dot, u.command, this);
  
    // Publish parameters
    paramActualPosition->set(actualPositionTemp);
    paramActualWorldPosition->set(actualWorldPositionTemp);
    paramErrorQuat->set(errorQuatTemp);

    // If we are able to grab the lock on visualizationPublisher,
    // publish the visualization markers.
    if (visualizationPublisher.trylock())
    {
        // Create a marker showing the goal vector

        // startPoint.x = tran_BodyToBase[0]; startPoint.y = tran_BodyToBase[1]; startPoint.z =tran_BodyToBase[2];
        // endPoint.x = tran_BodyToBase[0]+goalHeading(0); endPoint.y = tran_BodyToBase[1]+goalHeading(1); endPoint.z = tran_BodyToBase[2]+goalHeading(2);

        visualizationPublisher.msg_.markers[0].header.stamp = ros::Time::now();
        
        bodyToBaseTrans = RigidBodyDynamics::CalcBodyToBaseCoordinates(model.rbdlModel(), Q, bodyId_, base_vector, false);

        visualizationPublisher.msg_.markers[0].pose.position.x = bodyToBaseTrans[0];
        visualizationPublisher.msg_.markers[0].pose.position.y = bodyToBaseTrans[1];
        visualizationPublisher.msg_.markers[0].pose.position.z = bodyToBaseTrans[2];

        // This is for testing purposes
        visualizationPublisher.msg_.markers[0].pose.orientation.x = markerOrientation[0];
        visualizationPublisher.msg_.markers[0].pose.orientation.y = markerOrientation[1];
        visualizationPublisher.msg_.markers[0].pose.orientation.z = markerOrientation[2];
        visualizationPublisher.msg_.markers[0].pose.orientation.w = markerOrientation[3];

        visualizationPublisher.msg_.markers[0].scale.x = markerLength;
        visualizationPublisher.msg_.markers[0].scale.y = markerWidth;
        visualizationPublisher.msg_.markers[0].scale.z = markerHeight;

        visualizationPublisher.msg_.markers[0].color.a = markerColorGoal[0];

        visualizationPublisher.msg_.markers[0].color.r = markerColorGoal[1];
        visualizationPublisher.msg_.markers[0].color.g = markerColorGoal[2];
        visualizationPublisher.msg_.markers[0].color.b = markerColorGoal[3];
    
        // Create a marker showing the current heading
        // startPoint.x =0; startPoint.y=0;startPoint.z=0;
        // endPoint.x = bodyFrameVector_(0); endPoint.y = bodyFrameVector_(1); endPoint.z = bodyFrameVector_(2);
        
        // currMarker.header.frame_id = "world";
        // currMarker.header.stamp = ros::Time::now();
        // currMarker.ns = getTypeName() + "/" + getInstanceName();
        // currMarker.id = 1;
        // currMarker.type = visualization_msgs::Marker::ARROW;
        // currMarker.action = visualization_msgs::Marker::ADD;
        // currMarker.points.push_back(startPoint);
        // currMarker.points.push_back(endPoint);
        // currMarker.scale.x = 0.02; // arrow shaft diameter
        // currMarker.scale.y = 0.08; // arrow head diamater
        // currMarker.scale.z = 0.0;
        // currMarker.color.a = 1.0;
        // currMarker.color.r = 0.0;
        // currMarker.color.g = 0.0;
        // currMarker.color.b = 1.0;
    
        // Publish the MarkerArray
        visualizationPublisher.unlockAndPublish();
    }

    return true;
}

} // namespace task_library
} // namespace controlit
