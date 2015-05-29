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
#include <controlit/constraint_library/FlatContactConstraint.hpp>

namespace controlit {
namespace constraint_library  {

FlatContactConstraint::FlatContactConstraint() :
    Constraint("controlit/FlatContactConstraint", "__UNNAMED_FLAT_CONTACT_CONSTRAINT_INSTANCE__")
{
    this->constrainedDOFs_ = 6;
    // Declare our parameters
    setupParameters();
}

void FlatContactConstraint::init(RigidBodyDynamics::Model& robot)
{
    // Resize temporary variables
    Jpv.resize(3, robot.dof_count);
    Jpw.resize(3, robot.dof_count);

    // Parent class init() must be called otherwise initialization won't be complete!
    Constraint::init(robot);
}

void FlatContactConstraint::getJacobian(RigidBodyDynamics::Model& robot,
    const Vector& Q, Matrix& Jc)
{
    // Check size of incoming Jc
    assert(isInitialized() && Jc.rows() == 6 && Jc.cols() == (int)robot.dof_count);

    // Since flag set to false, ZeroQ shouldn't be used in calculation
    RigidBodyDynamics::CalcPointJacobian(robot,
                                         Q,
                                         masterNode_,
                                         cp1_,
                                         Jpv,
                                         false);

    // Since flag set to false, ZeroQ shouldn't be used in calculation
    RigidBodyDynamics::CalcPointJacobianW(robot,
                                          Q,
                                          masterNode_,
                                          cp1_,
                                          Jpw,
                                          false);

    // Set Jc
    Jc.topRows(3) = Jpv;
    Jc.bottomRows(3) = Jpw;

    // Publish Jc
    localJcParam->set(Jc);
}

void FlatContactConstraint::setupParameters()
{
    // Parent class version must be called first
    Constraint::setupParameters();

    // Now declare my parameters
    declareParameter("contactPoint", &cp1_);
}


} // constraint_library
} // namespace controlit
