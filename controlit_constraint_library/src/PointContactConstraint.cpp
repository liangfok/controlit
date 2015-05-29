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

#include <controlit/constraint_library/PointContactConstraint.hpp>

namespace controlit {
namespace constraint_library {

//! Constructors
PointContactConstraint::PointContactConstraint() :
  controlit::Constraint("controlit/PointContactConstraint", "__UNAMED_CONSTRAINT__")
{
    this->constrainedDOFs_ = 3;
    setupParameters();
}

void PointContactConstraint::init(RigidBodyDynamics::Model& robot)
{
    //assert(!isInitialized());

    // Parent class init() must be called otherwise initialization won't be complete!
    Constraint::init(robot);
}

void PointContactConstraint::getJacobian(RigidBodyDynamics::Model& robot, const Vector& Q, Matrix& Jc)
{
    assert(isInitialized() && Jc.rows() == 3 && Jc.cols() == (int)robot.dof_count);

    RigidBodyDynamics::CalcPointJacobian(robot,
                      Q,
                      masterNode_,
                      cp1_,
                      Jc,
                      false);
}

void PointContactConstraint::setupParameters()
{
    // Parent class version must be called first
    Constraint::setupParameters();
  
    // Declare our parameters
    declareParameter("contactPoint", &cp1_);
}

} // namespace constraint_library
} // namespace controlit
