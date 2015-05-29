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

#ifndef __CONTROLIT_CONTROL_MODEL_LIBRARY_HPP__
#define __CONTROLIT_CONTROL_MODEL_LIBRARY_HPP__

#include <controlit/ContactConstraint.hpp>
#include <RigidBodyDynamics/Extras/rbdl_extras.hpp>
#include <controlit/addons/eigen/LinearAlgebra.hpp>
#include <rbdl/rbdl.h>

namespace controlit {

using RigidBodyDynamics::Math::SpatialVector;
using RigidBodyDynamics::Math::Xtrans;
// using RigidBodyDynamics::Math::Vector3d;

struct ControlModelLibrary
{
  static controlit::ControlModel * getLegAndFootModel(std::shared_ptr<controlit::RobotState> robotState)
  {
    // Create some links.
    // The parameters are as follows: mass, center of mass, radius of gyration.
    // The center of mass and radius of gyration are given in the body's coordinate frame.
    RigidBodyDynamics::Body body1(1, Vector3d(0, 0, 0),   Vector3d(1, 1, 1));
    RigidBodyDynamics::Body body2(1, Vector3d(0, 0, 0.5), Vector3d(1, 1, 1));
    RigidBodyDynamics::Body body3(1, Vector3d(0, 0, 0.5), Vector3d(1, 1, 1));

    // Create some joints.
    //  - The first joint is a 6-DOF joint.  The first three joints are prismatic
    //    tx, ty, tz, and the second three joints are revolute rx, ry, yz.
    //    Note: multi-DOF joints in RBDL are actually implemented by N single DOF
    //    joints with massless bodies!!!
    //  - The parameters for the second and third joints are as follows: jointtype, axis
    RigidBodyDynamics::Joint joint1(SpatialVector(0, 0, 0, 1, 0, 0),
                                    SpatialVector(0, 0, 0, 0, 1, 0),
                                    SpatialVector(0, 0, 0, 0, 0, 1),
                                    SpatialVector(1, 0, 0, 0, 0, 0),
                                    SpatialVector(0, 1, 0, 0, 0, 0),
                                    SpatialVector(0, 0, 1, 0, 0, 0));
    RigidBodyDynamics::Joint joint2(RigidBodyDynamics::JointTypeRevolute, Vector3d(1, 0, 0));
    RigidBodyDynamics::Joint joint3(RigidBodyDynamics::JointTypeRevolute, Vector3d(0, 1, 0));

    // Create and initialize a RBDL robot model for the WBC Controller.
    RigidBodyDynamics::Model * model = new RigidBodyDynamics::Model();
    model->Init();


    // Add the bodies and joints to the RBDL model.
    // The parameters are as follows: joint frame, joint, body, joint name
    // The joint frame is defined in terms of the previous body's frame.
    //
    //         ---------
    //         |       |
    //         | body3 |              body3 COM at: (0, 0, 1.5)
    //         |       |
    //         ---------
    //             |
    //             x                  joint3 at: 0, 0, 1
    //             |
    //         ---------
    //         |       |
    //         | body2 |              body2 COM at: (0, 0, 1)
    //         |       |
    //         ---------
    //             |
    //             x                  joint2 at: (0, 0, 0.5)
    //             |
    //    -------------------
    //    |      body1      |
    //    -------------------
    model->AppendBody(Xtrans(Vector3d(0, 0, 0.0)), joint1, body1, "rigid6DoF");
    model->AppendBody(Xtrans(Vector3d(0, 0, 0.5)), joint2, body2, "revolute1DoF_1");
    model->AppendBody(Xtrans(Vector3d(0, 0, 0.5)), joint3, body3, "revolute1DoF_2");

    // Create a map from the link name to the joint name.
    controlit::ControlModel::LinkNameToJointNameMap_t * linkNameToJointNameMap
      = new controlit::ControlModel::LinkNameToJointNameMap_t();
    linkNameToJointNameMap->insert(std::make_pair("body1", "rigid6DoF"));
    linkNameToJointNameMap->insert(std::make_pair("body2", "revolute1DoF_1"));
    linkNameToJointNameMap->insert(std::make_pair("body3", "revolute1DoF_2"));

    // Build a constraint set
    ConstraintSet * constraintSet = new ConstraintSet;

    robotState->init(model->dof_count - joint1.mDoFCount); // subtract the number of virtual joints

    // Create and initialize the control model
    controlit::ControlModel * controlModel = new controlit::ControlModel();
    controlModel->init(model, robotState, linkNameToJointNameMap, constraintSet);

    // Create some fake data
    // Vector Q(controlModel->rbdlModel().dof_count); Q.setZero(); Q(6) = 0.2;
    // Vector Qd(controlModel->rbdlModel().dof_count); Qd.setZero(); Qd(7) = 0.01;
    // Vector Qdd(controlModel->rbdlModel().dof_count); Qdd.setZero();

    return controlModel;
  }
};

} // namespace controlit

#endif




