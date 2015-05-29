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
#include <controlit/constraint_library/TransmissionConstraint.hpp>

namespace controlit {
namespace constraint_library {

TransmissionConstraint::TransmissionConstraint() :
  controlit::Constraint("controlit/TransmissionConstraint","__UNAMED_CONSTRAINT__")
{
    this->constrainedDOFs_ = 1;
    setupParameters();
}

void TransmissionConstraint::setupParameters()
{
    // setup parent class' parameters
    Constraint::setupParameters();

    // Declare our parameters
    declareParameter("slaveNodeName", & slaveNodeName_);
    declareParameter("transmissionRatio", & transmissionRatio);
}

void TransmissionConstraint::init(RigidBodyDynamics::Model& robot)
{
    Constraint::init(robot); // initialize parent class

    localJc.setZero(1, robot.dof_count);  // initialize size of constraint Jacobian
}

void TransmissionConstraint::getJacobian(RigidBodyDynamics::Model & robot, const Vector & Q, Matrix & Jc)
{
    // Ensure size of Jc is correct.
    assert(isInitialized() && Jc.rows() == 1 && Jc.cols() == (int)robot.dof_count);
  
    // Set Jc
    Jc.setZero();
    Jc(0, masterNode_ -1) = -transmissionRatio;
    Jc(0, slaveNode_ -1) = 1;

    // Publish Jc if it is bound
    localJcParam->set(Jc);
}

} // namespace constraint_library
} // namespace controlit
