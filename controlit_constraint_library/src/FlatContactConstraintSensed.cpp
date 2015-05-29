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
#include <controlit/constraint_library/FlatContactConstraintSensed.hpp>
#include <RigidBodyDynamics/Extras/rbdl_extras.hpp>
#include <vector>

namespace controlit {
namespace constraint_library {

FlatContactConstraintSensed::FlatContactConstraintSensed() :
  ContactConstraint("controlit/FlatContactConstraintSensed", "__UNNAMED__")
{
    CONTROLIT_PR_INFO << "Constructor called.";
  
    this->constrainedDOFs_ = 6;
  
    // 1 = local, 0 = global
    rxnFrFrame_ = 0;
    goalCOPFrame_ = 1;
    contactFrame_ = 1;
  
    // Declare our parameters
    setupParameters();
}

FlatContactConstraintSensed::FlatContactConstraintSensed(std::string const& name) :
    ContactConstraint("controlit/FlatContactConstraintSensed", name)
{
    CONTROLIT_PR_INFO << "Constructor called.";
  
    this->constrainedDOFs_ = 6;
  
    //1 = local, 0 = global
    rxnFrFrame_ = 0;
    goalCOPFrame_ = 1;
    contactFrame_ = 1;
  
    // Declare our parameters
    setupParameters();
}

void FlatContactConstraintSensed::init(RigidBodyDynamics::Model& robot)
{
    CONTROLIT_PR_INFO << "Method called.";
  
    //Set up phi_ mapping
    phi_.setIdentity(6,6);
  
    // Resize temporary variables
    Jpv.resize(3, robot.dof_count);
    Jpw.resize(3, robot.dof_count);
  
    // Resize local variables
    worldCOP_.resize(3);
    localCOP_.resize(3);
    rxnForceCOM_.resize(6);
  
    // Parent class init() must be called otherwise initialization won't be complete!
    ContactConstraint::init(robot);
}

void FlatContactConstraintSensed::getJacobian(RigidBodyDynamics::Model& robot, const Vector& Q, Matrix& Jc)
{
    // Calculate COP
    if(rxnForceCOM_.norm() > 0) // Safeguard against not recieving information
    {
        // Compute the COP in the world frame
        if(rxnFrFrame_ == 0) // The reaction forces are specified in the world frame
        {
            if(contactFrame_ == 1) // The contact normal is specified in the local frame
                RigidBodyDynamics::Extras::calcCOPfromFrCOM(robot, Q, masterNode_,
                    rxnForceCOM_, contactNormal_, contactPlanePoint_, worldCOP_,
                    RigidBodyDynamics::Extras::Frame::WORLD,  // frame of reaction forces
                    RigidBodyDynamics::Extras::Frame::LOCAL,  // frame of contact normal
                    RigidBodyDynamics::Extras::Frame::LOCAL,  // frame of contact plane point
                    RigidBodyDynamics::Extras::Frame::WORLD); // frame of returned COP value
            else // The contact normal is specified in the world frame
                RigidBodyDynamics::Extras::calcCOPfromFrCOM(robot, Q, masterNode_,
                    rxnForceCOM_, contactNormal_, contactPlanePoint_, worldCOP_,
                    RigidBodyDynamics::Extras::Frame::WORLD,  // frame of reaction forces
                    RigidBodyDynamics::Extras::Frame::WORLD,  // frame of contact normal
                    RigidBodyDynamics::Extras::Frame::WORLD,  // frame of contact plane point
                    RigidBodyDynamics::Extras::Frame::WORLD); // frame of returned COP value
        }
        else // The reaction forces are specified in the local frame
        {
            if(contactFrame_ == 1) // The contact normal is specified in the local frame
                RigidBodyDynamics::Extras::calcCOPfromFrCOM(robot, Q, masterNode_,
                    rxnForceCOM_, contactNormal_, contactPlanePoint_, worldCOP_,
                    RigidBodyDynamics::Extras::Frame::LOCAL,  // frame of reaction forces
                    RigidBodyDynamics::Extras::Frame::LOCAL,  // frame of contact normal
                    RigidBodyDynamics::Extras::Frame::LOCAL,  // frame of contact plane point
                    RigidBodyDynamics::Extras::Frame::WORLD); // frame of returned COP value
            else // The contact normal is specified in the world frame
                RigidBodyDynamics::Extras::calcCOPfromFrCOM(robot, Q, masterNode_,
                    rxnForceCOM_, contactNormal_, contactPlanePoint_, worldCOP_,
                    RigidBodyDynamics::Extras::Frame::LOCAL,  // frame of reaction forces
                    RigidBodyDynamics::Extras::Frame::WORLD,  // frame of contact normal
                    RigidBodyDynamics::Extras::Frame::WORLD,  // frame of contact plane point
                    RigidBodyDynamics::Extras::Frame::WORLD); // frame of returned COP value
        }
    
        // Compute the COP in the local frame
        localCOP_ = RigidBodyDynamics::CalcBaseToBodyCoordinates(robot, Q, masterNode_, worldCOP_, false);
    }
    else // no sensor data...uses reasonable guess of COP from the model
    {
        worldCOP_ = RigidBodyDynamics::CalcBodyToBaseCoordinates(robot, Q, masterNode_, localCOP_, false);
    }
  
    if(goalWorldCOP_.size() != 3) goalWorldCOP_.resize(3);
  
    if(goalCOPFrame_ == 1) // 1 means local frame
    {
        // The goal COP was specified in the local frame.
        // Compute the goal COP in the world frame.
        goalWorldCOP_ = RigidBodyDynamics::CalcBodyToBaseCoordinates(robot, Q, masterNode_, goalLocalCOP_, false);
    }
    else
    {
        // The goal COP was specified in the world frame.
        // Compute the goal COP in the local frame.
        goalLocalCOP_ = RigidBodyDynamics::CalcBaseToBodyCoordinates(robot, Q, masterNode_, goalWorldCOP_, false);
    }
  
    ContactConstraint::publishParameters();
  
    // Check size of incoming Jc
    assert(isInitialized() && Jc.rows() == 6 && Jc.cols() == (int)robot.dof_count);
  
    RigidBodyDynamics::CalcPointJacobian(robot,
                                         Q,
                                         masterNode_,
                                         goalLocalCOP_,
                                         Jpv,
                                         false);
  
    RigidBodyDynamics::CalcPointJacobianW(robot,
                                          Q,
                                          masterNode_,
                                          goalLocalCOP_,
                                          Jpw,
                                          false);
  
    // Set Jc
    Jc.topRows(3) = Jpv;
    Jc.bottomRows(3) = Jpw;
}

void FlatContactConstraintSensed::setupParameters()
{
    // Parent class version must be called first
    ContactConstraint::setupParameters();
}

} // namespace constraint_library
} // namespace controlit
