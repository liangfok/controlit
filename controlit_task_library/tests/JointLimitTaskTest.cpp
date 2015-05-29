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
#include <controlit/task_library/JointLimitTask.hpp>
#include <controlit/ControlModelLibrary.hpp>
#include <controlit/RobotState.hpp>

namespace controlit {
namespace task_library {

class JointLimitTaskTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    ros::Time::init();
    robotState.reset(new controlit::RobotState());
    controlit::ControlModel * controlModel = controlit::ControlModelLibrary::getLegAndFootModel(robotState);
    myModel.reset(controlModel);
  }

  virtual void TearDown()
  {
    myModel.reset();
  }

  std::shared_ptr<controlit::RobotState> robotState;
  std::unique_ptr<controlit::ControlModel> myModel;
  controlit::Parameter * p; // A pointer to a parameter
};

TEST_F(JointLimitTaskTest, BasicTest)
{
  std::unique_ptr<controlit::Task> task(new controlit::task_library::JointLimitTask);

  // Verify the number of actuable joints is as expected.
  int NactuatedJoints = myModel->getNActuableDOFs();
  ASSERT_EQ(NactuatedJoints, 2) << "Incorrect number of actuated joints: Got "
    << NactuatedJoints << " expected 2";

  // Set the parameter upperStopRad to be all zeros
  p = task->lookupParameter("upperStopRad");
  EXPECT_TRUE(p) << "Unable to get upperStopRad parameter.";
  Vector upperStopRad(8); upperStopRad.setZero();
  EXPECT_TRUE(p->set(upperStopRad));

  // Set the parameter upperTriggerRad to be a tolerance higher than the
  // upperStopRad parameter.  In this case, we set it to be:
  // <1e12, 1e12,  1e12, 1e12, 1e12, 1e12, 0.1, 0.1>
  p = task->lookupParameter("upperTriggerRad");
  EXPECT_TRUE(p) << "Unable to get upperTriggerRad parameter.";
  Vector upperTriggerRad(8); upperTriggerRad.setZero();
  upperTriggerRad(6) = 0.1; upperTriggerRad(7) = 0.1;
  for(int i = 0; i < 6; i++)
    upperTriggerRad(i) = 1e12;
  EXPECT_TRUE(p->set(upperTriggerRad));

  // Set the parameter lowerStopRad to be:
  // <0, 0, 0, 0, 0, 0, -0.2, -0.2>
  p = task->lookupParameter("lowerStopRad");
  EXPECT_TRUE(p) << "Unable to get lowerStopRad parameter.";
  Vector lowerStopRad(8); lowerStopRad.setZero();
  lowerStopRad(6) = -0.2; lowerStopRad(7) = -0.2;
  EXPECT_TRUE(p->set(lowerStopRad));

  // Set the parameter lowerTriggerRad to be  a tolerance lower than the
  // lowerStopRad parameter.  In this case, we set it to be:
  // <-1e12, -1e12,  -1e12, -1e12, -1e12, -1e12, -0.3, -0.3>
  p = task->lookupParameter("lowerTriggerRad");
  EXPECT_TRUE(p) << "Unable to get lowerTriggerRad parameter.";
  Vector lowerTriggerRad(8); lowerTriggerRad.setZero();
  lowerTriggerRad(6) = -0.3; lowerTriggerRad(7) = -0.3;
  for(int i = 0; i < 6; i++)
    lowerTriggerRad(i) = -1e12;
  EXPECT_TRUE(p->set(lowerTriggerRad));

  // Set parameter kp to be ones
  p = task->lookupParameter("kp");
  EXPECT_TRUE(p) << "Unable to get kp parameter.";
  Vector kp(8); kp.setOnes();
  EXPECT_TRUE(p->set(kp));

  // Set parameter kd to be zeros
  p = task->lookupParameter("kd");
  EXPECT_TRUE(p) << "Unable to get kd parameter.";
  Vector kd(8); kd.setZero();
  EXPECT_TRUE(p->set(kd));

  // Set parameter maxVelocity to be zeros
  p = task->lookupParameter("maxVelocity");
  EXPECT_TRUE(p) << "Unable to get kd parameter.";
  Vector maxVel(8); maxVel.setZero();
  EXPECT_TRUE(p->set(maxVel));

  // Initialize the robot model
  EXPECT_TRUE(task->init(*myModel));

  // Update the inactive state within the task
  task->updateState(myModel.get());

  // Now that the inactive state is updated, make it the active state.
  task->checkUpdatedState();

  // Get the task's Jacobian matrix
  Matrix Jtask;
  task->getJacobian(Jtask);

  // Verify that the task's Jacobian matrix has the right dimensions.
  // In this case, since the task's joints are all at the zero position,
  // we expect the Jacobian matrix to have zero rows.
  ASSERT_EQ(Jtask.rows(), 0)
    << "Jtask has incorrect number of rows: expected 1 got "
    << Jtask.rows();

  ASSERT_EQ(Jtask.cols(), myModel->getNumDOFs())
    << "Task jacobian has incorrect number of columns: expected "
    << myModel->getNumDOFs() << " got " << Jtask.cols();

  TaskCommand command;
  task->getCommand(*myModel, command);
  EXPECT_TRUE(command.command.size() == 0)
    << "Error wrong command size. It was " << command.command.size() << ", expected 0";
  EXPECT_TRUE(command.type == CommandType::ACCELERATION);

  // Change the joint state to be: <0, 0, 0, 0, 0, 0, -0.4, 0.3>.
  // The last two joints have reached their limits.
  Vector Q(myModel->getNActuableDOFs());
  Q << -0.4, 0.3;
  Vector Qd(Q), Qdd(Q);

  for (int ii = 0; ii < Q.size(); ii++)
  {
    robotState->setJointPosition(ii, Q[ii]);
    robotState->setJointVelocity(ii, Qd[ii]);
    robotState->setJointAcceleration(ii, Qdd[ii]);
  }

  myModel->updateJointState();
  myModel->update();

  // Update the inactive state within the task
  task->updateState(myModel.get());

  // Now that the inactive state is updated, make it the active state.
  task->checkUpdatedState();

  task->getJacobian(Jtask);

  ASSERT_EQ(Jtask.rows(), 2)
    << "Jtask has incorrect number of rows: expected 2 got "
    << Jtask.rows();

  ASSERT_EQ(Jtask.cols(), myModel->getNumDOFs())
    << "Task jacobian has incorrect number of columns: expected "
    << myModel->getNumDOFs() << " got " << Jtask.cols();

  task->getCommand(*myModel, command);

  EXPECT_TRUE(command.command.size() == 2)
    << "Error wrong command size. It was " << command.command.size();

  EXPECT_TRUE(command.type == CommandType::ACCELERATION);
}

} // namespace task_library
} // namespace controlit
