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

#ifndef _CONTROLIT_CONSTRAINT_LIBRARY_DERIVED_TEST_CLASSES_HPP_
#define _CONTROLIT_CONSTRAINT_LIBRARY_DERIVED_TEST_CLASSES_HPP_

#include <controlit/constraint_library/ConstraintLibrary.hpp>

using namespace RigidBodyDynamics::Math;

struct TestFlatContactConstraint : public controlit::constraint_library::FlatContactConstraint
{
  TestFlatContactConstraint(std::string const& masterNode, Vector3d const& pt) :
    controlit::constraint_library::FlatContactConstraint()
  {
    this->masterNodeName_ = masterNode;
    this->cp1_ = pt;
  }
};

struct TestFlatContactConstraintSensed : public controlit::constraint_library::FlatContactConstraintSensed
{
  TestFlatContactConstraintSensed(std::string const& masterNode, Vector3d const& pt,
                            Vector3d const& contactNormal, Vector3d const& contactPlanePoint,
                            VectorNd const& rxnForceCOM, Vector3d const& COP) :
    controlit::constraint_library::FlatContactConstraintSensed(masterNode)
  {
    this->masterNodeName_ = masterNode;
    this->localCOP_ = pt;
    this->goalLocalCOP_ = pt;
    this->contactNormal_ = contactNormal;
    this->contactPlanePoint_ = contactPlanePoint;
    this->rxnForceCOM_ = rxnForceCOM;
    this->worldCOP_ = COP;
  }
};

struct TestPointContactConstraint : public controlit::constraint_library::PointContactConstraint
{
  TestPointContactConstraint(std::string const& masterNode, Vector3d const& pt) :
    controlit::constraint_library::PointContactConstraint()
  {
    this->masterNodeName_ = masterNode;
    this->cp1_ = pt;
  }
};

struct TestTransmissionConstraint : public controlit::constraint_library::TransmissionConstraint
{
  TestTransmissionConstraint(std::string const& masterNode, std::string const& slaveNode, double c) :
    controlit::constraint_library::TransmissionConstraint()
  {
    this->masterNodeName_ = masterNode;
    this->slaveNodeName_ = slaveNode;
    this->c_ = c;
  }
};

#endif
