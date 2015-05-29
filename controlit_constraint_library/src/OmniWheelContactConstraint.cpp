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
#include <controlit/constraint_library/OmniWheelContactConstraint.hpp>

namespace controlit {
namespace constraint_library  {

OmniWheelContactConstraint::OmniWheelContactConstraint() :
    Constraint("controlit/OmniWheelContactConstraint", "__UNNAMED_OmniWheel_CONTACT_CONSTRAINT_INSTANCE__")
{
    this->constrainedDOFs_ = 2;
    // Declare our parameters
    setupParameters();
}

void OmniWheelContactConstraint::init(RigidBodyDynamics::Model& robot)
{
    // Resize temporary variables
    Jpv.resize(3, robot.dof_count);
    Jpw.resize(3, robot.dof_count);
    zeroVec.setZero(3);
    referenceFrameRotation.setIdentity(3,3);
    worldFrameNormalAxis.setZero();
    worldFrameWheelAxis.setZero();
    worldFrameContactPoint.setZero();
    worldFrameWheelCenter.setZero();
    worldFrameRollingDirection.setZero();


    // Look up reference frame index
    referenceFrameNode_ = robot.GetBodyId(referenceFrameName_.c_str());
    if (referenceFrameNode_ == std::numeric_limits<unsigned int>::max())
    {
        std::stringstream sstr;
        sstr << "init: reference frame body " << referenceFrameName_ << " not found! Aborting initialization. Valid body names: " << std::endl;
  
        // Get valid body names
        for (auto const& it : robot.mBodyNameMap)
        {
            sstr << it.first << "(" << it.second << ")" << std::endl;
        }
  
        CONTROLIT_PR_ERROR << sstr.str();
  
        return;
    }

    normalAxis_.normalize();
    wheelAxis_.normalize();

    // Parent class init() must be called otherwise initialization won't be complete!
    Constraint::init(robot);
}

void OmniWheelContactConstraint::getJacobian(RigidBodyDynamics::Model& robot,
  const Vector& Q, Matrix& Jc)
{
    // Check size of incoming Jc
    assert(isInitialized() && Jc.rows() == 2 && Jc.cols() == (int)robot.dof_count);

    // Since flag set to false, ZeroQ shouldn't be used in calculation
    RigidBodyDynamics::CalcPointJacobian(robot,
                                         Q,
                                         masterNode_,
                                         zeroVec,
                                         Jpv,
                                         false);

    // Since flag set to false, ZeroQ shouldn't be used in calculation
    RigidBodyDynamics::CalcPointJacobianW(robot,
                                          Q,
                                          masterNode_,
                                          zeroVec,
                                          Jpw,
                                          false);


    referenceFrameRotation = RigidBodyDynamics::CalcBodyWorldOrientation(robot,Q,referenceFrameNode_,false).transpose();
    worldFrameNormalAxis = referenceFrameRotation * normalAxis_;
    worldFrameWheelAxis = referenceFrameRotation * wheelAxis_;
    worldFrameRollingDirection = worldFrameNormalAxis.cross(worldFrameWheelAxis);
    // worldFrameRollingDirection = referenceFrameRotation * wheelAxis_;
    // worldFrameWheelAxis = worldFrameNormalAxis.cross(worldFrameRollingDirection);
    worldFrameContactPoint = RigidBodyDynamics::CalcBodyToBaseCoordinates(robot,Q,referenceFrameNode_,cp1_,false);
    worldFrameWheelCenter = RigidBodyDynamics::CalcBodyToBaseCoordinates(robot,Q,masterNode_,zeroVec,false);

    // Set Jc
    Jc.topRows(1) = (worldFrameRollingDirection).transpose() *
                    (Jpv - RigidBodyDynamics::Math::VectorCrossMatrix(worldFrameContactPoint - worldFrameWheelCenter) * Jpw);
    Jc.bottomRows(1) = (worldFrameNormalAxis).transpose() * Jpv;
}

void OmniWheelContactConstraint::setupParameters()
{
    // Parent class version must be called first
    Constraint::setupParameters();

    // Now declare my parameters
    declareParameter("referenceFrameName", &referenceFrameName_);
    declareParameter("contactPoint", &cp1_);
    declareParameter("normalAxis", &normalAxis_);
    declareParameter("wheelAxis", &wheelAxis_);
}


} // constraint_library
} // namespace controlit
