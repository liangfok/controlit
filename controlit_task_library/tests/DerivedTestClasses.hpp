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

#ifndef __CONTROLIT_TASK_LIBRARY_DERIVED_TEST_CLASSES_HPP_
#define __CONTROLIT_TASK_LIBRARY_DERIVED_TEST_CLASSES_HPP_

#include <controlit/ContactConstraint.hpp>
#include <RigidBodyDynamics/Extras/rbdl_extras.hpp>
#include <controlit/addons/eigen/LinearAlgebra.hpp>
#include <rbdl/rbdl.h>

using namespace RigidBodyDynamics::Math;

struct TestFlatContactConstraintSensed : public controlit::ContactConstraint
{
    TestFlatContactConstraintSensed(std::string const& masterNode, Vector3d const& pt,
                              Vector3d const& contactNormal, Vector3d const& contactPlanePoint,
                              VectorNd const& rxnForceCOM, Vector3d const& COP) :
        controlit::ContactConstraint("wbc/TestFlatContactConstraintSensed", "__UNNAMED__")
    {
        phi_.setIdentity(6,6);
        this->constrainedDOFs_ = 6;
        rxnFrFrame_ = 0;
        ContactConstraint::setupParameters();
    
        this->masterNodeName_ = masterNode;
        this->localCOP_ = pt;
        this->contactNormal_ = contactNormal;
        this->contactPlanePoint_ = contactPlanePoint;
        this->rxnForceCOM_ = rxnForceCOM;
        this->worldCOP_ = COP;
    }
  
    virtual void
    init(RigidBodyDynamics::Model& robot)
    {
        assert(!isInitialized());
        //Set up phi_ mapping
        phi_.setIdentity(6,6);
  
        // Resize temporary variables
        Jpv.resize(3, robot.dof_count);
        Jpw.resize(3, robot.dof_count);
  
        // Parent class init() must be called otherwise initialization won't be complete!
        Constraint::init(robot);
    }
  
    virtual std::string
    getMasterNodeName()
    {
        return masterNodeName_;
    }
  
    virtual void
    getJacobian(RigidBodyDynamics::Model& robot, const Vector& Q, Matrix& Jc)
    {
        // Calculate COP
        if(rxnFrFrame_==0) //know the reaction forces in world frame
          RigidBodyDynamics::Extras::calcCOPfromFrCOM(robot, Q, masterNode_,
                         rxnForceCOM_, contactNormal_, contactPlanePoint_, worldCOP_,
                         RigidBodyDynamics::Extras::Frame::WORLD, //frame of reaction forces
                         RigidBodyDynamics::Extras::Frame::LOCAL, //frame of contact normal
                         RigidBodyDynamics::Extras::Frame::LOCAL, //frame of contact plane point
                         RigidBodyDynamics::Extras::Frame::WORLD); //frame of returned COP value
        else //know the reaction forces in local frame
          RigidBodyDynamics::Extras::calcCOPfromFrCOM(robot, Q, masterNode_,
                         rxnForceCOM_, contactNormal_, contactPlanePoint_, worldCOP_,
                         RigidBodyDynamics::Extras::Frame::LOCAL, //frame of reaction forces
                         RigidBodyDynamics::Extras::Frame::LOCAL, //frame of contact normal
                         RigidBodyDynamics::Extras::Frame::LOCAL, //frame of contact plane point
                         RigidBodyDynamics::Extras::Frame::WORLD); //frame of returned COP value
  
        p = this->lookupParameter("worldCOP");
        s = p->set(worldCOP_);
  
        localCOP_ = RigidBodyDynamics::CalcBaseToBodyCoordinates(robot,Q,masterNode_,worldCOP_,false);
  
        p = this->lookupParameter("localCOP");
        s = p->set(localCOP_);
  
  
        Vector rxnForceCOP_(6);
  
        Vector worldCOM_(3);
        worldCOM_ = RigidBodyDynamics::CalcBaseToBodyCoordinates(robot,Q,masterNode_,robot.mBodies[masterNode_].mCenterOfMass,false);
  
        //TODO: double check wrench transformation--sign error. NOTE: also effectos calcCOP methods
        rxnForceCOP_.head(3) = rxnForceCOM_.head(3);
        rxnForceCOP_.tail(3) = rxnForceCOM_.tail(3) + RigidBodyDynamics::Math::VectorCrossMatrix(rxnForceCOM_.head(3))*(worldCOP_-worldCOM_);
  
  
        // Check size of incoming Jc
        assert( isInitialized() && (Jc.rows()==6) && (Jc.cols()==robot.dof_count) );
  
        // Since flag set to false, Q shouldn't be used in calculation
        RigidBodyDynamics::CalcPointJacobian(robot,
                                             Q,
                                             masterNode_,
                                             localCOP_,
                                             Jpv,
                                             false);
  
        // Since flag set to false, Q shouldn't be used in calculation
        RigidBodyDynamics::CalcPointJacobianW(robot,
                                              Q,
                                              masterNode_,
                                              localCOP_,
                                              Jpw,
                                              false);
  
        // Set Jc
        Jc.topRows(3) = Jpv;
        Jc.bottomRows(3) = Jpw;
    }
  
    virtual Matrix
    getPhi(const Matrix3d& Rn, const Matrix3d& Rb){ return phi_; }
  
    //! For setting parameters
    controlit::Parameter * p;
  
    //! Storage for temporary calculations
    Matrix Jpv, Jpw;

};

#endif




