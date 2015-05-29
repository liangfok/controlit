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

#include <memory>
#include <gtest/gtest.h>
#include <rbdl/rbdl.h>
#include <controlit/constraint_library/ConstraintLibrary.hpp>
#include <controlit/ConstraintSet.hpp>
#include "DerivedTestClasses.hpp"

using controlit::Constraint;
using controlit::ConstraintSet;
using Matrix;
using Vector;
using RigidBodyDynamics::Math::Vector3d;
using RigidBodyDynamics::Math::SpatialVector;

class ConstraintSetTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        myRobot.reset( new RigidBodyDynamics::Model() );
        myRobot->Init();
    
        //RBDL Body(mass, center of mass, raduis of gyration)
        RigidBodyDynamics::Body body1(1.0, Vector3d(0.0, 0.0, 0.0), Vector3d(1.0, 1.0, 1.0));
        RigidBodyDynamics::Body body2(1.0, Vector3d(0.0, 0.0, 0.5), Vector3d(1.0, 1.0, 1.0));
        RigidBodyDynamics::Body body3(1.0, Vector3d(0.0, 0.0, 0.5), Vector3d(1.0, 1.0, 1.0));
    
        //RBDL Joint(jointtype, axis)
        //Note: multi-DOF joints in RBDL are actually implemented by N single DOF joints with massless bodies!!!
        RigidBodyDynamics::Joint joint1(SpatialVector(0.0,0.0,0.0,1.0,0.0,0.0),
                        SpatialVector(0.0,0.0,0.0,0.0,1.0,0.0),
                        SpatialVector(0.0,0.0,0.0,0.0,0.0,1.0),
                        SpatialVector(1.0,0.0,0.0,0.0,0.0,0.0),
                        SpatialVector(0.0,1.0,0.0,0.0,0.0,0.0),
                        SpatialVector(0.0,0.0,1.0,0.0,0.0,0.0));
        RigidBodyDynamics::Joint joint2(RigidBodyDynamics::JointTypeRevolute, Vector3d(1.0, 0.0, 0.0));
        RigidBodyDynamics::Joint joint3(RigidBodyDynamics::JointTypeRevolute, Vector3d(0.0, 1.0, 0.0));
        myRobot->AppendBody(Xtrans(Vector3d(0.0,0.0,0.0)), joint1,body1, "rigid6DoF");
        myRobot->AppendBody(Xtrans(Vector3d(0.0,0.0,0.5)), joint2,body2, "revolute1DoF_1");
        myRobot->AppendBody(Xtrans(Vector3d(0.0,0.0,0.5)), joint3,body3, "revolute1DoF_2");
    }
  
    virtual void TearDown()
    {
        myRobot.reset();
    }
  
    std::unique_ptr<RigidBodyDynamics::Model> myRobot;
};

/* ----------------------------------------------------------------------------
 *  Constraint set tests
 * --------------------------------------------------------------------------*/
TEST_F(ConstraintSetTest, ConstructorTest)
{
    Constraint* PCC( new TestPointContactConstraint("rigid6DoF", Vector3d(0.5,1.0,1.0)) );
    std::unique_ptr<ConstraintSet> CS( new ConstraintSet("pointcontact_constraint") );
    CS->addConstraint(PCC);
    CS->init( *(myRobot.get()) );
  
    EXPECT_TRUE(CS->getNConstrainedDOFs()==3);
    EXPECT_TRUE(CS->getNConstraints()==1);
    // EXPECT_TRUE( CS->isConstrained(myRobot->GetBodyId("rigid6DoF")) );
}

TEST_F(ConstraintSetTest, InitTest1)
{
    Constraint* TC(new TestTransmissionConstraint( "revolute1DoF_1", //careful using id
                            "revolute1DoF_2", //careful using id
                            2.0) );
    std::unique_ptr<ConstraintSet> CS( new ConstraintSet("transmission_constraint") );
    CS->addConstraint(TC);
    CS->init( *(myRobot.get()) );
  
    EXPECT_TRUE(CS->getNConstrainedDOFs()==1);
    EXPECT_TRUE(CS->getNConstraints()==1);
    // EXPECT_FALSE(CS->isConstrained(1));
  
    Matrix U(myRobot->dof_count - CS->getNumUnactuatedDof() - CS->getNumVirtualDof(), myRobot->dof_count);
    CS->getU(U);
  
    Matrix expectedU;
    expectedU.setZero(myRobot->dof_count - CS->getNumUnactuatedDof() - CS->getNumVirtualDof(), myRobot->dof_count);
    expectedU(0, 6) = 1;
  
    // TODO: Verify by hand that expectedU is correct
    EXPECT_TRUE(U == expectedU);
  
    Matrix Jc(CS->getNConstrainedDOFs(), myRobot->dof_count);
    CS->getJacobian(Jc);
  
    Matrix expectedJc;
    expectedJc.setZero(CS->getNConstrainedDOFs(), myRobot->dof_count);
  
    //std::cout<<"Jc is : \n" <<Jc<<"\n expected : \n"<<expectedJc<<std::endl;
  
    // TODO: Verify by hand that expectedJc is correct
    EXPECT_TRUE(Jc == expectedJc);
}

TEST_F(ConstraintSetTest, InitTest2)
{
    Constraint* TC( new TestFlatContactConstraint( "rigid6DoF", //careful using id
                               Vector3d(0.0,0.0,0.0)) );
    std::unique_ptr<ConstraintSet> CS( new ConstraintSet() );
    CS->addConstraint(TC);
    CS->init( *(myRobot.get()) );
  
    //spot checks
    EXPECT_TRUE(CS->getNConstrainedDOFs()==6);
    EXPECT_TRUE(CS->getNConstraints()==1);
    // EXPECT_FALSE(CS->isConstrained(8));
  
    Matrix U(myRobot->dof_count - CS->getNumUnactuatedDof() - CS->getNumVirtualDof(), myRobot->dof_count);
    CS->getU(U);
  
    Matrix expectedU;
    expectedU.setZero(myRobot->dof_count - CS->getNumUnactuatedDof() - CS->getNumVirtualDof(), myRobot->dof_count);
    expectedU(0,6) = 1;
    expectedU(1,7) = 1;
  
    // TODO: Verify by hand that expectedU is correct
    EXPECT_TRUE(U == expectedU);
  
    Matrix Jc(CS->getNConstrainedDOFs(), myRobot->dof_count);
    CS->getJacobian(Jc);
  
    Matrix expectedJc;
    expectedJc.setZero(CS->getNConstrainedDOFs(), myRobot->dof_count);
  
    // TODO: Verify by hand that expectedJc is correct
    EXPECT_TRUE(Jc == expectedJc);
}


TEST_F(ConstraintSetTest, MultiConstraintTest)
{
    Constraint* TC( new TestFlatContactConstraint( "rigid6DoF", //careful using id
                             Vector3d(0.0,0.0,0.0)) );
    Constraint* PC( new TestPointContactConstraint( "revolute1DoF_2",
                            Vector3d(0.5,1.0,1.0)) );
  
    std::unique_ptr<ConstraintSet> CS( new ConstraintSet("6") );
    CS->addConstraint(TC);
    CS->addConstraint(PC);
    CS->init( *(myRobot.get()) );
  
    //spot checks (yes, this is over constrained, but whatever.)
    EXPECT_TRUE(CS->getNConstrainedDOFs()==9);
    EXPECT_TRUE(CS->getNConstraints()==2);
    // EXPECT_TRUE(CS->isConstrained(8));
  
    Matrix U(myRobot->dof_count - CS->getNumUnactuatedDof() - CS->getNumVirtualDof(), myRobot->dof_count);
    CS->getU(U);
  
    Matrix expectedU;
    expectedU.setZero(myRobot->dof_count - CS->getNumUnactuatedDof() - CS->getNumVirtualDof(), myRobot->dof_count);
    expectedU(0,6) = 1;
    expectedU(1,7) = 1;
  
    // TODO: Verify by hand that expectedU is correct
    EXPECT_TRUE(U == expectedU);
  
    Matrix Jc(CS->getNConstrainedDOFs(),myRobot->dof_count);
    CS->getJacobian(Jc);
  
    Matrix expectedJc;
    expectedJc.setZero(CS->getNConstrainedDOFs(),myRobot->dof_count);
  
    // TODO: Verify by hand that expectedJc is correct
    EXPECT_TRUE(Jc == expectedJc);
}

TEST_F(ConstraintSetTest, MathTest)
{
    Constraint* TC = new TestFlatContactConstraint("rigid6DoF", //careful using id
                             Vector3d(0.0,0.0,0.0));
    std::unique_ptr<ConstraintSet> CS( new ConstraintSet() );
    CS->addConstraint(TC);
    CS->init( *(myRobot.get()) );
  
    //spot checks
    EXPECT_TRUE(CS->getNConstrainedDOFs()==6);
    EXPECT_TRUE(CS->getNConstraints()==1);
    // EXPECT_FALSE(CS->isConstrained(8));
  
    Matrix U(myRobot->dof_count - CS->getNumUnactuatedDof() - CS->getNumVirtualDof(), myRobot->dof_count);
    CS->getU(U);
  
    Matrix expectedU;
    expectedU.setZero(myRobot->dof_count - CS->getNumUnactuatedDof() - CS->getNumVirtualDof(), myRobot->dof_count);
    expectedU(0,6) = 1;
    expectedU(1,7) = 1;
  
    // TODO: Verify by hand that expectedU is correct
    EXPECT_TRUE(U == expectedU);
  
    Matrix Jc(CS->getNConstrainedDOFs(), myRobot->dof_count);
    CS->getJacobian(Jc);
  
    Matrix expectedJc;
    expectedJc.setZero(CS->getNConstrainedDOFs(), myRobot->dof_count);
  
    // TODO: Verify by hand that expectedJc is correct
    EXPECT_TRUE(Jc == expectedJc);
  
    Matrix A(myRobot->dof_count, myRobot->dof_count);
    Vector Q(myRobot->dof_count); Q.setZero(); Q(7) = 0.3;
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(*myRobot, Q, A);
  
    Matrix Ainv = A.inverse(); //Is A ever singular?
    CS->update(*myRobot,Q,Ainv);
  
    Matrix JcBar;
    CS->getJacobianBar(JcBar);
  
    Matrix expectedJcBar;
    expectedJcBar.setZero(8, 6); // 8 rows, 6 columns
  
    // TODO: Verify by hand that expectedJcBar is correct
    // EXPECT_TRUE(JcBar == expectedJcBar);
  
    Matrix Nc;
  
    CS->getNc(Nc);
  
    Matrix expectedNc;
    expectedNc.setZero(8, 8);
    expectedNc(0,0) = 1;
    expectedNc(1,1) = 1;
    expectedNc(2,2) = 1;
    expectedNc(3,3) = 1;
    expectedNc(4,4) = 1;
    expectedNc(5,5) = 1;
    expectedNc(6,6) = 1;
    expectedNc(7,7) = 1;
  
    // TODO: Verify by hand that expectedNc is correct
    // EXPECT_TRUE(Nc == expectedNc);
  
    Matrix UNc;
    CS->getUNc(UNc);
  
    Matrix expectedUNc;
    expectedUNc.setZero(2,8);
    expectedUNc(0,6) = 1;
    expectedUNc(1,7) = 1;
  
    // TODO: Verify by hand that expectedUNc is correct
    //EXPECT_TRUE(UNc == expectedUNc);
  
    Matrix UNcBar;
    CS->getUNcBar(UNcBar);
  
    Matrix expectedUNcBar;
    expectedUNcBar.setZero(8, 2);
    expectedUNcBar(0,1) = 0.104529 - 2.1471e-7 - 1.80411e-15;
    expectedUNcBar(1,0) = -0.0513579 - 3.05192e-8 - 2.09971e-14;
    expectedUNcBar(2,1) = 0.0335241 - 1.87916e-08 + 4.15917e-14;
    expectedUNcBar(3,0) = -0.658416 - 1.01922e-07 + 2.56462e-13;
    expectedUNcBar(4,1) = -0.319355 - 4.6084e-07 + 9.2168e-07 + 3.72313e-13;
    expectedUNcBar(5,0) = 0.00274342 - 4.44949e-09 + 8.89898e-09 - 3.38661e-15;
    expectedUNcBar(6,0) = 1;
    expectedUNcBar(7,1) = 1;
  
    // TODO: Verify by hand that expectedUNcBar is correct
    // EXPECT_TRUE(UNcBar == expectedUNcBar);
}

TEST_F(ConstraintSetTest, NullConstraintTest)
{
    std::unique_ptr<ConstraintSet> CS( new ConstraintSet() );
    CS->init( *(myRobot.get()) ); //No constraints, but 6 Rigid body DOFs.
                       //Unrealistic, but might be useful for only partial pinning to the world
  
    EXPECT_TRUE(CS->getNConstrainedDOFs() == 0);
    EXPECT_TRUE(CS->getNConstraints() == 0);
    EXPECT_TRUE(CS->getNumVirtualDof() == 6) << "RigidBodyDof: " << CS->getNumVirtualDof();
  
    Matrix U(myRobot->dof_count - CS->getNumUnactuatedDof() - CS->getNumVirtualDof(), myRobot->dof_count);
    CS->getU(U);
  
    Matrix expectedU;
    expectedU.setZero(myRobot->dof_count - CS->getNumUnactuatedDof() - CS->getNumVirtualDof(), myRobot->dof_count);
    expectedU(0,6) = 1;
    expectedU(1,7) = 1;
  
    // TODO: Verify by hand that expectedU is correct
    EXPECT_TRUE(U == expectedU);
  
    Matrix A(myRobot->dof_count, myRobot->dof_count);
    Vector Q(myRobot->dof_count); Q.setZero(); Q(7) = 0.3;
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(*myRobot, Q, A);
  
    Matrix Ainv = A.inverse(); //Is A ever singular?
    Matrix JcBar;
    CS->getJacobianBar(JcBar);
  
    Matrix expectedJcBar;
    expectedJcBar.setZero(8,8);
    expectedJcBar(0,0) = 1;
    expectedJcBar(1,1) = 1;
    expectedJcBar(2,2) = 1;
    expectedJcBar(3,3) = 1;
    expectedJcBar(4,4) = 1;
    expectedJcBar(5,5) = 1;
    expectedJcBar(6,6) = 1;
    expectedJcBar(7,7) = 1;
  
    // TODO: Verify by hand that expectedJcBar is correct
    // EXPECT_TRUE(JcBar == expectedJcBar);
  
    Matrix Nc;
  
    CS->getNc(Nc);
  
    Matrix expectedNc = expectedJcBar;
  
    // TODO: Verify by hand that expectedNc is correct
    // EXPECT_TRUE(Nc == expectedNc);
  
    Matrix UNc;
    CS->getUNc(UNc);
  
    Matrix expectedUNc = U;
  
    // TODO: Verify by hand that expectedUNc is correct
    // EXPECT_TRUE(UNc == U);
  
    Matrix UNcBar;
    CS->getUNcBar(UNcBar);
  
    Matrix expectedUNcBar;
    expectedUNcBar.setZero(8, 2);
    expectedUNcBar(0,1) = 0.104529 - 2.1471e-7 - 1.80411e-15;
    expectedUNcBar(1,0) = -0.0513579 - 3.05192e-8 - 2.09971e-14;
    expectedUNcBar(2,1) = 0.0335241 - 1.87916e-08 + 4.15917e-14;
    expectedUNcBar(3,0) = -0.658416 - 1.01922e-07 + 2.56462e-13;
    expectedUNcBar(4,1) = -0.319355 - 4.6084e-07 + 9.2168e-07 + 3.72313e-13;
    expectedUNcBar(5,0) = 0.00274342 - 4.44949e-09 + 8.89898e-09 - 3.38661e-15;
    expectedUNcBar(6,0) = 1;
    expectedUNcBar(7,1) = 1;
  
    // TODO: Verify by hand that expectedUNcBar is correct
    //EXPECT_TRUE(UNcBar == expectedUNcBar);
}
