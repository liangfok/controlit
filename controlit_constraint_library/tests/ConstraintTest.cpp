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

#include <gtest/gtest.h>
#include <rbdl/rbdl.h>
#include <controlit/constraint_library/ConstraintLibrary.hpp>
#include <controlit/ConstraintSet.hpp>

#include "DerivedTestClasses.hpp"

using Matrix;
using Vector;
using controlit::Constraint;

class ConstraintTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        ros::Time::init();
        myRobot.reset(new RigidBodyDynamics::Model());
        myRobot->Init();
    }
    virtual void TearDown()
    {
        myRobot.reset();
    }
  
    std::unique_ptr<RigidBodyDynamics::Model> myRobot;
};

/* ----------------------------------------------------------------------------
 *  Constraint tests
 * --------------------------------------------------------------------------*/
TEST_F(ConstraintTest, PointContactConstraintTest)
{
    //RBDL Body(mass, center of mass, raduis of gyration)
    RigidBodyDynamics::Body b1(1.0, Vector3d(0.0, 0.0, -0.5), Vector3d(1.0, 1.0, 1.0));
  
    //RBDL Joint(jointtype, axis)
    RigidBodyDynamics::Joint j1(RigidBodyDynamics::JointTypeRevolute, Vector3d(1.0, 0.0, 0.0));
    myRobot->AddBody(0, Xtrans(Vector3d(0.0,0.0,0.0)), j1, b1, "joint_1");
  
    std::unique_ptr<Constraint> PCC( new TestPointContactConstraint("joint_1", Vector3d(0.5,1.0,1.0)) );
    PCC->init( *(myRobot.get()) );
  
    Matrix PJc1(PCC->getNConstrainedDOFs(), myRobot->dof_count); //WARNING: Matrix Must be the right size!! (check:NDoF includes fixed joints?)
    Vector Q1(myRobot->dof_count);
    PCC->getJacobian(*(myRobot.get()), Q1, PJc1);
    //std::cout<<PJc1<<std::endl;
    EXPECT_TRUE(PJc1(0,0) == 0);//Jacobian should NOT depend on the axis of rotation
  
    //RBDL Body(mass, center of mass, radius of gyration)
    RigidBodyDynamics::Body b2(1.0, Vector3d(0.0, 0.0, -0.5), Vector3d(1.0, 1.0, 1.0));
  
    //RBDL Joint(jointtype, axis)
    RigidBodyDynamics::Joint j2(RigidBodyDynamics::JointTypeRevolute, Vector3d(1.0, 0.0, 0.0));
    myRobot->AddBody(0,Xtrans(Vector3d(0.0,0.0,0.0)), j2, b2, "joint_2");
  
    Vector Q2(myRobot->dof_count);
    Matrix PJc2(PCC->getNConstrainedDOFs(), myRobot->dof_count); //WARNING: Matrix Must be the right size!! (check:NDoF includes fixed joints?)
    PCC->getJacobian(*(myRobot.get()), Q2, PJc2);
    //std::cout<<PJc2<<std::endl;
    EXPECT_TRUE(PJc2(0,1) == 0);//Jacobian should NOT depend on the axis of rotation
}

TEST_F(ConstraintTest, FlatContactConstraintTest)
{
    // RBDL Body(mass, center of mass, raduis of gyration)
    RigidBodyDynamics::Body body1(1.0, Vector3d(0.0, 0.0, 0.0), Vector3d(1.0, 1.0, 1.0));
    RigidBodyDynamics::Body body2(1.0, Vector3d(0.0, 0.0, 0.5), Vector3d(1.0, 1.0, 1.0));
    RigidBodyDynamics::Body body3(1.0, Vector3d(0.0, 0.0, 0.5), Vector3d(1.0, 1.0, 1.0));
  
    // RBDL Joint(jointtype, axis)
    // Note: multi-DOF joints in RBDL are actually implemented by N single DOF joints with massless bodies!!!
    RigidBodyDynamics::Joint joint1(SpatialVector(0.0,0.0,0.0,1.0,0.0,0.0),
                    SpatialVector(0.0,0.0,0.0,0.0,1.0,0.0),
                    SpatialVector(0.0,0.0,0.0,0.0,0.0,1.0),
                    SpatialVector(1.0,0.0,0.0,0.0,0.0,0.0),
                    SpatialVector(0.0,1.0,0.0,0.0,0.0,0.0),
                    SpatialVector(0.0,0.0,1.0,0.0,0.0,0.0));
    RigidBodyDynamics::Joint joint2(RigidBodyDynamics::JointTypeRevolute, Vector3d(1.0, 0.0, 0.0));
    RigidBodyDynamics::Joint joint3(RigidBodyDynamics::JointTypePrismatic, Vector3d(0.0, 1.0, 0.0));
    myRobot->AppendBody(Xtrans(Vector3d(0.0,0.0,0.0)), joint1, body1,"rigid6DoF");
    myRobot->AppendBody(Xtrans(Vector3d(0.0,0.0,0.5)), joint2, body2,"revolute1DoF_1");
    myRobot->AppendBody(Xtrans(Vector3d(0.0,0.0,0.5)), joint3, body3,"revolute1DoF_2");
  
    std::unique_ptr<Constraint> C( new TestFlatContactConstraint("revolute1DoF_2", //careful using id
                                   Vector3d(0.0,0.0,0.0)) );
    C->init(*(myRobot.get()));
  
    Matrix TJc1(C->getNConstrainedDOFs(), myRobot->dof_count); //WARNING: Matrix Must be the right size!! (check:NDoF includes fixed joints?)
    Vector Q1(myRobot->dof_count);
    C->getJacobian(*(myRobot.get()), Q1, TJc1);
  
    // Create an 'expectedResult' matrix that looks like:
    // 1 0 0 0 0 0 0 0
    // 0 1 0 0 0 0 0 1
    // 0 0 1 0 0 0 0 0
    // 0 0 0 1 0 0 1 0
    // 0 0 0 0 1 0 0 0
    // 0 0 0 0 0 1 0 0
    Matrix expectedResult;
    expectedResult.setZero(6, 8); // 6 rows 8 columns
    expectedResult(0,0) = 1;
    expectedResult(1,1) = 1;
    expectedResult(2,2) = 1;
    expectedResult(3,3) = 1;
    expectedResult(4,4) = 1;
    expectedResult(5,5) = 1;
    expectedResult(3,6) = 1;
    expectedResult(1,7) = 1;
  
    // TODO: Verify by hand that TJc1 is correct
    EXPECT_TRUE(TJc1 == expectedResult);
}

TEST_F(ConstraintTest, FlatContactConstraintSensedTest)
{
    // RBDL Body(mass, center of mass, raduis of gyration)
    RigidBodyDynamics::Body body1(1.0, Vector3d(0.0, 0.0, 0.0), Vector3d(1.0, 1.0, 1.0));
    RigidBodyDynamics::Body body2(1.0, Vector3d(0.0, 0.0, 0.5), Vector3d(1.0, 1.0, 1.0));
    RigidBodyDynamics::Body body3(1.0, Vector3d(0.0, 0.0, 0.5), Vector3d(1.0, 1.0, 1.0));
  
    // RBDL Joint(jointtype, axis)
    // Note: multi-DOF joints in RBDL are actually implemented by N single DOF joints with massless bodies!!!
    RigidBodyDynamics::Joint joint1(SpatialVector(0.0,0.0,0.0,1.0,0.0,0.0),
                    SpatialVector(0.0,0.0,0.0,0.0,1.0,0.0),
                    SpatialVector(0.0,0.0,0.0,0.0,0.0,1.0),
                    SpatialVector(1.0,0.0,0.0,0.0,0.0,0.0),
                    SpatialVector(0.0,1.0,0.0,0.0,0.0,0.0),
                    SpatialVector(0.0,0.0,1.0,0.0,0.0,0.0));
    RigidBodyDynamics::Joint joint2(RigidBodyDynamics::JointTypeRevolute, Vector3d(1.0, 0.0, 0.0));
    RigidBodyDynamics::Joint joint3(RigidBodyDynamics::JointTypePrismatic, Vector3d(0.0, 1.0, 0.0));
    myRobot->AppendBody(Xtrans(Vector3d(0.0,0.0,0.0)), joint1, body1,"rigid6DoF");
    myRobot->AppendBody(Xtrans(Vector3d(0.0,0.0,0.5)), joint2, body2,"revolute1DoF_1");
    myRobot->AppendBody(Xtrans(Vector3d(0.0,0.0,0.5)), joint3, body3,"revolute1DoF_2");
  
    VectorNd rxnFr(6); rxnFr.setZero();
    rxnFr(2) = 10;
  
    std::unique_ptr<Constraint> C( new TestFlatContactConstraintSensed("revolute1DoF_2", //careful using id
                                   Vector3d(0.0,0.0,0.0), Vector3d(0,0,1), Vector3d(0,1,0), rxnFr, Vector3d(0.0,0.0,0.0)));
    C->init( *(myRobot.get()) );
  
    Matrix TJc1(C->getNConstrainedDOFs(), myRobot->dof_count); //WARNING: Matrix Must be the right size!! (check:NDoF includes fixed joints?)
    Vector Q1(myRobot->dof_count);
    C->getJacobian(*(myRobot.get()), Q1, TJc1);
  
    // Create an 'expectedResult' matrix that looks like:
    // 1 0 0 0 0 0 0 0
    // 0 1 0 0 0 0 0 1
    // 0 0 1 0 0 0 0 0
    // 0 0 0 1 0 0 1 0
    // 0 0 0 0 1 0 0 0
    // 0 0 0 0 0 1 0 0
    Matrix expectedResult;
    expectedResult.setZero(6, 8); // 6 rows 8 columns
    expectedResult(0,0) = 1;
    expectedResult(1,1) = 1;
    expectedResult(2,2) = 1;
    expectedResult(3,3) = 1;
    expectedResult(4,4) = 1;
    expectedResult(5,5) = 1;
    expectedResult(3,6) = 1;
    expectedResult(1,7) = 1;
  
    // TODO: Verify by hand that TJc1 is correct
    EXPECT_TRUE(TJc1 == expectedResult);
}

TEST_F(ConstraintTest, TransmissionConstraintTest)
{
    // RBDL Body(mass, center of mass, raduis of gyration)
    RigidBodyDynamics::Body body1(1.0, Vector3d(0.0, 0.0, 0.0), Vector3d(1.0, 1.0, 1.0));
    RigidBodyDynamics::Body body2(1.0, Vector3d(0.0, 0.0, 0.5), Vector3d(1.0, 1.0, 1.0));
    RigidBodyDynamics::Body body3(1.0, Vector3d(0.0, 0.0, 0.5), Vector3d(1.0, 1.0, 1.0));
  
    // RBDL Joint(jointtype, axis)
    // Note: multi-DOF joints in RBDL are actually implemented by N single DOF joints with massless bodies!!!
    RigidBodyDynamics::Joint joint1(SpatialVector(0.0,0.0,0.0,1.0,0.0,0.0),
                    SpatialVector(0.0,0.0,0.0,0.0,1.0,0.0),
                    SpatialVector(0.0,0.0,0.0,0.0,0.0,1.0),
                    SpatialVector(1.0,0.0,0.0,0.0,0.0,0.0),
                    SpatialVector(0.0,1.0,0.0,0.0,0.0,0.0),
                    SpatialVector(0.0,0.0,1.0,0.0,0.0,0.0));
    RigidBodyDynamics::Joint joint2(RigidBodyDynamics::JointTypeRevolute, Vector3d(1.0, 0.0, 0.0));
    RigidBodyDynamics::Joint joint3(RigidBodyDynamics::JointTypeRevolute, Vector3d(0.0, 1.0, 0.0));
    myRobot->AppendBody(Xtrans(Vector3d(0.0,0.0,0.0)),joint1,body1,"rigid6DoF");
    myRobot->AppendBody(Xtrans(Vector3d(0.0,0.0,0.5)),joint2,body2,"revolute1DoF_1");
    myRobot->AppendBody(Xtrans(Vector3d(0.0,0.0,0.5)),joint3,body3,"revolute1DoF_2");
  
    std::unique_ptr<Constraint> TC(new TestTransmissionConstraint("revolute1DoF_1", //careful using id
                                      "revolute1DoF_2", //careful using id
                                      2.0) );
    TC->init(*(myRobot.get()));
  
    Matrix TJc1(TC->getNConstrainedDOFs(), myRobot->dof_count); //WARNING: Matrix Must be the right size!! (check:NDoF includes fixed joints?)
    Vector Q1(myRobot->dof_count);
    TC->getJacobian(*(myRobot.get()), Q1, TJc1);
    //std::cout<<TJc1<<std::endl;
    EXPECT_TRUE(TJc1(0, myRobot->GetBodyId("revolute1DoF_1") - 1) == -2.0);
    EXPECT_TRUE(TJc1(0, myRobot->GetBodyId("revolute1DoF_2") - 1) == 1.0);
}

