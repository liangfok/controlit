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

#include <controlit/task_library/COMTask.hpp>
#include <RigidBodyDynamics/Extras/rbdl_extras.hpp>

namespace controlit {
namespace task_library {

// Uncomment one of the following lines to enable/disable detailed debug statements.
#define PRINT_DEBUG_STATEMENT(ss)
// #define PRINT_DEBUG_STATEMENT(ss) CONTROLIT_PR_DEBUG << ss;

#define PRINT_DEBUG_STATEMENT_RT(ss)
// #define PRINT_DEBUG_STATEMENT_RT(ss) CONTROLIT_PR_DEBUG_RT << ss;

COMTask::COMTask() :
  	controlit::LatchedTask("__UNNAMED_COMTask_TASK__", CommandType::ACCELERATION, new TaskState(), new TaskState()),
    paramFrameProjectedCOM(NULL),
    paramFrameProjectedCOMVel(NULL),
    paramWorldProjectedCOM(NULL),
    paramWorldProjectedCOMVel(NULL),
    paramWorldCOM(NULL),
    paramWorldCOMVel(NULL)
{
    // Declare the parameters
    declareParameter("goalPosition", &goalPosition_);
    declareParameter("goalVelocity", &goalVelocity_);
    declareParameter("projection", &projection_);

    paramFrameProjectedCOM = declareParameter("projectedPosition", &frameProjectedCOM_);
    paramWorldProjectedCOM = declareParameter("projectedWorldPosition", &worldProjectedCOM_);
    paramFrameProjectedCOMVel = declareParameter("projectedVelocity", &frameProjectedCOMVel_);
    paramWorldProjectedCOMVel = declareParameter("projectedWorldVelocity", &worldProjectedCOMVel_);
    paramWorldCOM = declareParameter("actualWorldPosition", &worldCOM_);
    paramWorldCOMVel = declareParameter("actualWorldVelocity", &worldCOMVel_);

    // Create the PD controller
    controller.reset(PDControllerFactory::create(SaturationPolicy::ComponentWiseVel));
  
    // Add controller parameters to this task
    controller->declareParameters(this);
}

bool COMTask::init(ControlModel & model)
{
    // Initialize parent class (frameId lookup)
  
    // Abort if parent class fails to initialize
    if (!LatchedTask::init(model)) return false;
  
    // Allocate space for frame calculations
    JvFrame.resize(3, model.getNumDOFs());
    JwFrame.resize(3, model.getNumDOFs());
    Jcom.resize(3, model.getNumDOFs());
    JtLoc.resize(3, model.getNumDOFs());
    linkIndexMask.setZero(3, model.getNumDOFs());
  
    // Resize output/optional parameters
    frameProjectedCOM_.resize(3);
    frameProjectedCOMVel_.resize(3);
    worldProjectedCOM_.resize(3);
    worldProjectedCOMVel_.resize(3);
    worldCOM_.resize(3);
    worldCOMVel_.resize(3);
  
    controller->resize(3, getEnableState() == EnableState::SENSING);
  
    // Initialize variables used in the sense() method
    RFrame.resize(3, 3);
    TFrame.resize(3);
    VFrame.resize(3);
    Q.setZero(model.getNumDOFs());
    Qd.setZero(model.getNumDOFs());
    comPos.setZero(3);
    comVel.setZero(3);
    actualPos.setZero(3);
    actualVel.setZero(3);
    goalPos.setZero(3);
    goalVel.setZero(3);

    // Check whether the input parameters have the correct dimensions. 
    // If the enable state is sensing, use default values.
    if (goalPosition_.rows() != 3)
    {
        if (getEnableState() == EnableState::SENSING)
            goalPosition_.setZero(3);
        else
        {
            CONTROLIT_ERROR << "ERROR: COMTask::init: Goal position must have " << 3 << " dimensions, got " << goalPosition_.rows();
            return false;
        }
    }
  
    if (goalVelocity_.rows() != 3)
    {
        if (getEnableState() == EnableState::SENSING)
            goalVelocity_.setZero(3);
        else
        {
            CONTROLIT_ERROR << "ERROR: COMTask::init: Goal velocity must have " << 3 << " dimensions, got " << goalVelocity_.rows();
            return false;
        }
    }
  
    if (projection_.rows() != 3 || projection_.cols() != 3)
    {
        if (getEnableState() == EnableState::SENSING)
            projection_.setIdentity(3, 3);
        else
        {
            CONTROLIT_ERROR << "ERROR: COMTask::init: Projection matrix must have size 3 x 3, got " << projection_.rows() << " x " << projection_.cols();
            return false;
        }
    }
  
    // Allocate space for other local variables
    errpos.resize(3);
    errvel.resize(3);
  
    linkIndexList.clear();
  
    if (!jointNameList_.empty())
    {
        for (unsigned int ii = 0; ii < jointNameList_.size(); ii++)
        {
            int id;
            if (!model.getFrameID(jointNameList_[ii], id))
                return false;
      
            //TODO : MAKE THIS MORE GENERAL--NOT NECESSARILY 6 VIRTUAL DOFs
            //Recall that RBDL indexes are 1-indexed (0 is root), but Eigen in 0-indexed
            linkIndexList.push_back((unsigned int)id);
            if(id > 6)
                linkIndexMask.col(id - 1) = Vector::Ones(3);
            else //id 6 is the base body--account for virtual dofs
                linkIndexMask.topLeftCorner(3,6) = Matrix::Ones(3,6);
        }
    }
  
    addDefaultBindings();

    return Task::init(model);
}

void COMTask::addDefaultBindings()
{
    // CONTROLIT_INFO << "Method Called!\n"
    //     << "  - useDefaultBindings = " << useDefaultBindings;
    if (!useDefaultBindings) return;

    if (!hasBinding("goalPosition"))     addParameter(createROSInputBinding("goalPosition", "std_msgs/Float64MultiArray"));
    if (!hasBinding("goalVelocity"))     addParameter(createROSInputBinding("goalVelocity", "std_msgs/Float64MultiArray"));
    if (!hasBinding("kp"))               addParameter(createROSInputBinding("kp", "std_msgs/Float64MultiArray"));
    if (!hasBinding("kd"))               addParameter(createROSInputBinding("kd", "std_msgs/Float64MultiArray"));
    if (!hasBinding("enableState"))      addParameter(createROSInputBinding("enableState", "std_msgs/Int32"));
    if (!hasBinding("tare"))             addParameter(createROSInputBinding("tare", "std_msgs/Int32"));
    if (!hasBinding("projection"))       addParameter(createROSInputBinding("projection", "std_msgs/Float64MultiArray"));

    if (!hasBinding("error"))                   addParameter(createROSOutputBinding("error", "std_msgs/Float64MultiArray"));
    if (!hasBinding("errorDot"))                addParameter(createROSOutputBinding("errorDot", "std_msgs/Float64MultiArray"));
    if (!hasBinding("errorNorm"))               addParameter(createROSOutputBinding("errorNorm", "std_msgs/Float64"));
    if (!hasBinding("errorDotNorm"))            addParameter(createROSOutputBinding("errorDotNorm", "std_msgs/Float64"));
    if (!hasBinding("projectedPosition"))       addParameter(createROSOutputBinding("projectedPosition", "std_msgs/Float64MultiArray"));
    if (!hasBinding("projectedWorldPosition"))  addParameter(createROSOutputBinding("projectedWorldPosition", "std_msgs/Float64MultiArray"));
    if (!hasBinding("projectedVelocity"))       addParameter(createROSOutputBinding("projectedVelocity", "std_msgs/Float64MultiArray"));
    if (!hasBinding("projectedWorldVelocity"))  addParameter(createROSOutputBinding("projectedWorldVelocity", "std_msgs/Float64MultiArray"));
    if (!hasBinding("actualWorldPosition"))     addParameter(createROSOutputBinding("actualWorldPosition", "std_msgs/Float64MultiArray"));
    if (!hasBinding("actualWorldVelocity"))     addParameter(createROSOutputBinding("actualWorldVelocity", "std_msgs/Float64MultiArray"));
    if (!hasBinding("PDCommand"))               addParameter(createROSOutputBinding("PDCommand", "std_msgs/Float64MultiArray"));
}
bool COMTask::updateStateImpl(ControlModel * model, TaskState * taskState)
{
    PRINT_DEBUG_STATEMENT("Method called!")
  
    assert(model != nullptr);
    assert(taskState != nullptr);
  
    Matrix & taskJacobian = taskState->getJacobian();
  
    // Check if latched status has been updated
    updateLatch(model);
  
    if(taskJacobian.rows() != 3 || taskJacobian.cols() != (int)model->getNumDOFs())
        taskJacobian.resize(3, model->getNumDOFs());
  
    RigidBodyDynamics::Extras::calcRobotJvCOM(model->rbdlModel(), model->getQ(), linkIndexList, Jcom);
  
    if(frameId_ != -1) //NOT using world reference frame
    {
        if(!isLatched)
        {
            RFrame = RigidBodyDynamics::CalcBodyWorldOrientation(model->rbdlModel(), model->getQ(), frameId_, false);
            TFrame = RigidBodyDynamics::CalcBodyToBaseCoordinates(model->rbdlModel(), model->getQ(), frameId_, Vector::Zero(3), false);
      
            Vector comPos = RigidBodyDynamics::Extras::calcRobotCOM(model->rbdlModel(), model->getQ());
      
            RigidBodyDynamics::CalcPointJacobian(model->rbdlModel(), model->getQ(), frameId_, Vector::Zero(3), JvFrame, false);
            RigidBodyDynamics::CalcPointJacobianW(model->rbdlModel(), model->getQ(), frameId_, Vector::Zero(3), JwFrame, false);
      
            Vector ProjectedGoal = RFrame.transpose() * projection_ * goalPosition_;
            Vector ProjectedCOM = RFrame.transpose() * projection_ * RFrame * comPos;
            Vector ProjectedTFrame = RFrame.transpose() * projection_ * RFrame * TFrame;
      
            // Terms from time derivative of current location
            taskJacobian = RigidBodyDynamics::Math::VectorCrossMatrix(ProjectedCOM - ProjectedTFrame) * JwFrame;
            taskJacobian -= RFrame.transpose() * projection_ * RFrame * RigidBodyDynamics::Math::VectorCrossMatrix(goalPosition_ - TFrame) * JwFrame;
            taskJacobian += RFrame.transpose() * projection_ * RFrame * (Jcom - JvFrame);
      
            // Terms from time derivative of goal
            taskJacobian -= RigidBodyDynamics::Math::VectorCrossMatrix(ProjectedGoal) * JwFrame;
            taskJacobian += JvFrame;
        }
        else
            taskJacobian = latchedRotation.transpose() * projection_ * latchedRotation * Jcom;
    }
    else
        taskJacobian = projection_ * Jcom;
  
    if(!linkIndexList.empty())
    {
        taskJacobian.noalias() = taskJacobian.cwiseProduct(linkIndexMask);
    }
  
    return true;
}

bool COMTask::sense(ControlModel & model)
{
    // Get the latest joint state information
    model.getLatestFullState(Q, Qd);

    //Check if latched status has been updated
    updateLatch(&model);

    //Set local variable to be modified--this will end up as WORLD frame goals
    goalPos = goalPosition_;
    goalVel = goalVelocity_;

    // Compute the position and velocity of the COM
    comPos = RigidBodyDynamics::Extras::calcRobotCOM(model.rbdlModel(), Q, linkIndexList);
    comVel = RigidBodyDynamics::Extras::calcRobotCOMVel(model.rbdlModel(), Q, Qd, linkIndexList);

    // Publish parameters "worldCOM" and "worldCOMVel"
    paramWorldCOM->set(comPos);
    paramWorldCOMVel->set(comVel);

    if(frameId_ != -1)  // Using a robot frame--possibly latched
    {
        if(isLatched)  // latched, don't care about relative motion, just offset
        {
            RFrame = latchedRotation;
            TFrame = latchedTranslation;
            // Projected values in frameName_ frame
            actualPos = projection_ * RFrame * (comPos - TFrame);
            actualVel = projection_ * RFrame * comVel;
            // Values in world frame for error calculations
            comPos = RFrame.transpose() * actualPos + TFrame;
            comVel = RFrame.transpose() * actualVel;
            goalPos = RFrame.transpose() * projection_ * goalPosition_ + TFrame;
            goalVel = RFrame.transpose() * projection_ * goalVelocity_;
        }
        else // Not latched, care about relative motion with reference frame
        {
            VFrame = RigidBodyDynamics::CalcPointVelocity(model.rbdlModel(), Q, Qd, frameId_, Vector::Zero(3), false);
            RFrame = RigidBodyDynamics::CalcBodyWorldOrientation(model.rbdlModel(), Q, frameId_, false);
            TFrame = RigidBodyDynamics::CalcBodyToBaseCoordinates(model.rbdlModel(), Q, frameId_, Vector::Zero(3), false);
            // Projected values in frameName_ frame
            actualPos = projection_ * RFrame * (comPos - TFrame);
            actualVel = projection_ * RFrame * (comVel - VFrame);
            // Values in world frame for error calculations
            comPos = RFrame.transpose() * actualPos + TFrame;
            comVel = RFrame.transpose() * actualVel + VFrame;
            goalPos = RFrame.transpose() * projection_ * goalPosition_ + TFrame;
            goalVel = RFrame.transpose() * projection_ * goalVelocity_ + VFrame;
        }

        paramFrameProjectedCOM->set(actualPos); // sets local variable 'actualPosition'
        paramFrameProjectedCOMVel->set(actualVel);
    }
    else // Using fixed world reference frame--latching immaterial
    {
        comPos  = projection_ * comPos;
        goalPos = projection_ * goalPosition_;
        comVel  = projection_ * comVel;
        goalVel = projection_ * goalVelocity_;

        // publish parameter "actualPosition"
        paramFrameProjectedCOM->set(comPos);
        paramFrameProjectedCOMVel->set(comVel);
    }

    paramWorldProjectedCOM->set(comPos);
    paramWorldProjectedCOMVel->set(comVel);

    return true;
}

// This method is inhereted from PDTask, which is the parent class
bool COMTask::getCommand(ControlModel & model, TaskCommand & u)
{
    PRINT_DEBUG_STATEMENT_RT("Method called!")
  
    if (!sense(model)) return false;
    
    // getJacobian(JtLoc);
    errpos = goalPos - comPos;
    errvel = goalVel - comVel; //-JtLoc * model.getQd();

    // Set the command type
    u.type = commandType_;
  
    controller->computeCommand(errpos, errvel, u.command, this);

    return true;
}

} // namespace task_library
} // namespace controlit
