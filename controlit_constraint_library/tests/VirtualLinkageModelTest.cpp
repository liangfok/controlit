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
#include <controlit/VirtualLinkageModel.hpp>
#include <controlit/ConstraintSet.hpp>
#include "DerivedTestClasses.hpp"

using controlit::Constraint;
using controlit::ConstraintSet;
using Matrix;
using Vector;
using controlit::VirtualLinkageModel;
using RigidBodyDynamics::Math::Vector3d;
using RigidBodyDynamics::Math::SpatialVector;

class VirtualLinkageModelTest : public ::testing::Test
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
        RigidBodyDynamics::Body body4(1.0, Vector3d(0.0, 0.0, 0.5), Vector3d(1.0, 1.0, 1.0));
        RigidBodyDynamics::Body body5(1.0, Vector3d(0.0, 0.0, 0.5), Vector3d(1.0, 1.0, 1.0));
        RigidBodyDynamics::Body body6(1.0, Vector3d(0.0, 0.0, 0.5), Vector3d(1.0, 1.0, 1.0));
        RigidBodyDynamics::Body body7(1.0, Vector3d(0.0, 0.0, 0.5), Vector3d(1.0, 1.0, 1.0));
        RigidBodyDynamics::Body body8(1.0, Vector3d(0.0, 0.0, 0.5), Vector3d(1.0, 1.0, 1.0));
    
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
        RigidBodyDynamics::Joint joint4(RigidBodyDynamics::JointTypeRevolute, Vector3d(0.0, 0.0, 1.0));
        RigidBodyDynamics::Joint joint5(RigidBodyDynamics::JointTypeRevolute, Vector3d(1.0, 0.0, 0.0));
        RigidBodyDynamics::Joint joint6(RigidBodyDynamics::JointTypeRevolute, Vector3d(0.0, 1.0, 0.0));
        RigidBodyDynamics::Joint joint7(RigidBodyDynamics::JointTypeRevolute, Vector3d(0.0, 0.0, 1.0));
        RigidBodyDynamics::Joint joint8(RigidBodyDynamics::JointTypeRevolute, Vector3d(1.0, 0.0, 0.0));
        myRobot->AppendBody(Xtrans(Vector3d(0.0,0.0,0.0)), joint1,body1, "rigid6DoF");
        myRobot->AppendBody(Xtrans(Vector3d(0.0,0.0,0.5)), joint2,body2, "revolute1DoF_1");
        myRobot->AppendBody(Xtrans(Vector3d(0.0,0.0,0.5)), joint3,body3, "revolute1DoF_2");
        myRobot->AppendBody(Xtrans(Vector3d(0.0,0.0,0.5)), joint4,body4, "revolute1DoF_3");
        myRobot->AppendBody(Xtrans(Vector3d(0.0,0.0,0.5)), joint5,body5, "revolute1DoF_4");
        myRobot->AppendBody(Xtrans(Vector3d(0.0,0.0,0.5)), joint6,body6, "revolute1DoF_5");
        myRobot->AppendBody(Xtrans(Vector3d(0.0,0.0,0.5)), joint7,body7, "revolute1DoF_6");
        myRobot->AppendBody(Xtrans(Vector3d(0.0,0.0,0.5)), joint8,body8, "revolute1DoF_7");
    }
    virtual void TearDown()
    {
        myRobot.reset();
    }
  
    std::unique_ptr<RigidBodyDynamics::Model> myRobot;
};

/* ----------------------------------------------------------------------------
 *  Virtual Linkage Model tests
 * --------------------------------------------------------------------------*/
TEST_F(VirtualLinkageModelTest, InitTest0)
{
    std::unique_ptr<ConstraintSet> CS( new ConstraintSet("null_constraint_set") );
    CS->init( *(myRobot.get()) );
  
    EXPECT_TRUE(CS->getNConstrainedDOFs()==0);
    EXPECT_TRUE(CS->getNConstraints()==0);
  
    std::unique_ptr<VirtualLinkageModel> VLM( new VirtualLinkageModel("flatcontact_constraint") );
    VLM->init(*(myRobot.get()), *(CS.get()));
    //std::cout<<"VLM->getContactCount() = "<<VLM->getContactCount()<<std::endl;
  
    EXPECT_TRUE(VLM->getContactCount()==0);
    EXPECT_FALSE(VLM->exists());
  
    Vector Q(myRobot->dof_count); Q.setZero(); Q(myRobot->dof_count-1) = 1.507;
    Matrix A(myRobot->dof_count, myRobot->dof_count);
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(*myRobot, Q, A);
    Matrix Ainv = A.inverse(); //Is A ever singular?
  
    CS->update(*myRobot, Q, Ainv);
    VLM->update(*myRobot, *CS ,Q, Ainv);
  
    Matrix Wint;
    VLM->getWint(Wint);
  
    std::cout << "Wint = "<<Wint<<std::endl;
}

TEST_F(VirtualLinkageModelTest, InitTest1)
{
    Vector rxn1(6); rxn1(0) = 0.01; rxn1(1) = 0.01; rxn1(2) = 1.0; rxn1(3) = 0.002; rxn1(4) = 0.002; rxn1(5) = 0.002;
    Constraint* FCC1( new TestFlatContactConstraintSensed("rigid6DoF", Vector3d(0.0,0.0,-0.1), Vector3d(0.0,0.0,1.0), Vector3d(0.0,0.5,-0.1), rxn1, Vector3d(0.0,0.0,0.0))  );
    Constraint* FCC2( new TestFlatContactConstraintSensed("revolute1DoF_7", Vector3d(0.0,0.0,-0.1), Vector3d(0.0,0.0,1.0), Vector3d(0.0,0.5,-0.1), rxn1, Vector3d(0.0,0.0,0.0) ) );
    std::unique_ptr<ConstraintSet> CS( new ConstraintSet("flatcontact_constraints") );
    CS->addConstraint(FCC1);
    CS->addConstraint(FCC2);
    CS->init( *(myRobot.get()) );
  
    EXPECT_TRUE(CS->getNConstrainedDOFs()==12);
    EXPECT_TRUE(CS->getNConstraints()==2);
    // EXPECT_TRUE( CS->isConstrained(myRobot->GetBodyId("rigid6DoF")) );
  
    std::unique_ptr<VirtualLinkageModel> VLM( new VirtualLinkageModel("flatcontact_constraints") );
    VLM->init(*(myRobot.get()), *(CS.get()));
    //std::cout<<"VLM->getContactCount() = "<<VLM->getContactCount()<<std::endl;
  
    EXPECT_TRUE(VLM->getContactCount()==2);
  
    Vector Q(myRobot->dof_count); Q.setZero(); Q(myRobot->dof_count-1) = 1.507;
    Matrix A(myRobot->dof_count, myRobot->dof_count);
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(*myRobot, Q, A);
    Matrix Ainv = A.inverse(); //Is A ever singular?
  
    CS->update(*myRobot, Q, Ainv);
    VLM->update(*myRobot, *CS ,Q, Ainv);
  
    Matrix Wint;
    VLM->getWint(Wint);
  
    std::cout<<"Wint = "<<Wint<<std::endl;
}

TEST_F(VirtualLinkageModelTest, InitTest2)
{
    Vector rxn1(6); rxn1(0) = 0.01; rxn1(1) = 0.01; rxn1(2) = 1.0; rxn1(3) = 0.002; rxn1(4) = 0.002; rxn1(5) = 0.002;
    Constraint* FCC1( new TestFlatContactConstraintSensed("rigid6DoF", Vector3d(0.0,0.0,-0.1), Vector3d(0.0,0.0,1.0), Vector3d(0.0,0.5,-0.1), rxn1, Vector3d(0.0,0.0,0.0))  );
    Constraint* FCC2( new TestFlatContactConstraintSensed("revolute1DoF_7", Vector3d(0.0,0.0,-0.1), Vector3d(0.0,0.0,1.0), Vector3d(0.0,0.5,-0.1), rxn1, Vector3d(0.0,0.0,0.0) ) );
    Constraint* TC( new TestTransmissionConstraint("revolute1DoF_3", "revolute1DoF_4", 1.0) );
    std::unique_ptr<ConstraintSet> CS( new ConstraintSet("flatcontact_constraints") );
    CS->addConstraint(FCC1);
    CS->addConstraint(TC);
    CS->addConstraint(FCC2);
    CS->init( *(myRobot.get()) );
  
    EXPECT_TRUE(CS->getNConstrainedDOFs()==13);
    EXPECT_TRUE(CS->getNConstraints()==3);
    // EXPECT_TRUE( CS->isConstrained(myRobot->GetBodyId("rigid6DoF")) );
    // EXPECT_TRUE( CS->isConstrained(myRobot->GetBodyId("revolute1DoF_4")) );
  
  
    std::unique_ptr<VirtualLinkageModel> VLM( new VirtualLinkageModel("flatcontact_constraints") );
    VLM->init(*(myRobot.get()), *(CS.get()));
    //std::cout<<"VLM->getContactCount() = "<<VLM->getContactCount()<<std::endl;
  
    EXPECT_TRUE(VLM->getContactCount()==2);
  
    Vector Q(myRobot->dof_count); Q.setZero(); Q(myRobot->dof_count-1) = 1.507;
    Matrix A(myRobot->dof_count, myRobot->dof_count);
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(*myRobot, Q, A);
    Matrix Ainv = A.inverse(); //Is A ever singular?
  
    CS->update(*myRobot, Q, Ainv);
    VLM->update(*myRobot, *CS ,Q, Ainv);
  
    Matrix Wint;
    VLM->getWint(Wint);
  
    std::cout<<"Wint = "<<Wint<<std::endl;
}