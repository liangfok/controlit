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
#include <controlit/ControlModel.hpp>
#include <controlit/TaskCommand.hpp>
#include <controlit/ConstraintSet.hpp>
#include <controlit/task_library/GenericInternalForceTask.hpp>
#include <controlit/ControlModelLibrary.hpp>
#include "DerivedTestClasses.hpp"
#include <controlit/RobotState.hpp>

namespace controlit {
namespace task_library {

using RigidBodyDynamics::Math::Vector3d;

class GenericInternalForceTaskTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        ros::Time::init();
    
        // Create the control model.
        robotState.reset(new controlit::RobotState());
        controlit::ControlModel * controlModel = controlit::ControlModelLibrary::getLegAndFootModel(robotState);
        myModel.reset(controlModel);
    
        // Build a constraint.
        Vector rxn1(6); rxn1 << 0.01, 0.01, 1, 0.002, 0.002, 0.002;
        Constraint * flatContactConstraint
            = new TestFlatContactConstraintSensed("rigid6DoF",
                                                  Vector3d(0, 0, -0.1), Vector3d(0, 0, 1), Vector3d(0, 0.5, -0.1),
                                                  rxn1, Vector3d(0, 0, 0));
     
        // Add the constraint to the model's constraint set.
        myModel->constraints().addConstraint(flatContactConstraint);
    
        // Since the model's constraint set changed, re-initialize the model.
        myModel->reinit();
    
        // Set the joint state
        Vector Q(myModel->getNActuableDOFs());     Q << 0, 0.2;
        Vector Qd(myModel->getNActuableDOFs());    Qd << 0.01, 0; //setZero(); Qd(7) = 0.01;
        Vector Qdd(myModel->getNActuableDOFs());   Qdd.setZero();
    
        for (int ii = 0; ii < Q.size(); ii++)
        {
            robotState->setJointPosition(ii, Q[ii]);
            robotState->setJointVelocity(ii, Qd[ii]);
            robotState->setJointAcceleration(ii, Qdd[ii]);
        }
    
        // Since the joint state just changed, update the model.
        myModel->updateJointState();
        myModel->update();
    }
  
    virtual void TearDown()
    {
        myModel.reset();
    }
  
    std::shared_ptr<controlit::RobotState> robotState;
    std::unique_ptr<controlit::ControlModel> myModel;
    controlit::Parameter * p; // A pointer to a parameter
};

TEST_F(GenericInternalForceTaskTest, BasicTest)
{
    std::unique_ptr<controlit::Task> task(new controlit::task_library::GenericInternalForceTask);
  
    int NactuatedJoints = myModel->getNActuableDOFs();
    ASSERT_EQ(NactuatedJoints, 2) << "Incorrect number of actuated joints: Got "
        << NactuatedJoints << " expected 2";
  
    // Set the goal position parameter
    p = task->lookupParameter("goalFint");
    EXPECT_TRUE(p) << "Unable to get goalFint parameter.";
  
    // For a general internal tension task, Fint has size (number of contact pairs) + 3*(number of contacts)
    Vector goalFint(myModel->virtualLinkageModel().getWint().rows()); goalFint.setZero();
    EXPECT_TRUE(p->set(goalFint);
  
    // Init task
    EXPECT_TRUE(task->init(*myModel));
  
    // Update the inactive state within the task
    task->updateState(myModel.get());
  
    // Now that the inactive state is updated, make it the active state.
    task->checkUpdatedState();
  
    TaskCommand command;
    task->getCommand(*myModel, command);
  
    EXPECT_TRUE(command.type == CommandType::INTERNAL_FORCE)
      << "Error type of command, it should be an INTERNAL_FORCE command but got " << command.type;
  
    EXPECT_TRUE(command.command == goalFint) << "Error, I didn't get out what I put in";
  
    Matrix Jtask;
    task->getJacobian(Jtask);
    EXPECT_TRUE(Jtask.rows() == myModel->virtualLinkageModel().getWint().rows());
}

} // namespace task_library
} // namespace controlit
