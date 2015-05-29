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
#include <controlit/task_library/COMTask.hpp>
#include <controlit/ControlModelLibrary.hpp>
#include <controlit/RobotState.hpp>

namespace controlit {
namespace task_library {

class COMTaskTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
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

TEST_F(COMTaskTest, BasicTest)
{
  std::unique_ptr<controlit::Task> task(new controlit::task_library::COMTask);

  int NactuatedJoints = myModel->getNActuableDOFs();
  ASSERT_EQ(NactuatedJoints, 2) << "Incorrect number of actuated joints: Got "
    << NactuatedJoints << " expected 2";

  // Set the goal position parameter
  p = task->lookupParameter("goalPosition");
  EXPECT_TRUE(p) << "Unable to get goalPosition parameter.";
  Vector goalPos(3); goalPos.setZero();  // For a COMTask, the goal position as 3 dimensions: x, y, z
  EXPECT_TRUE(p->set(goalPos));

  // Set the goal velocity parameter
  p = task->lookupParameter("goalVelocity");
  EXPECT_TRUE(p) << "Unable to get goalVelocity parameter.";
  Vector goalVel(3); goalVel.setZero();  // For a COMTask, the goal position as 3 dimensions: x, y, z
  EXPECT_TRUE(p->set(goalVel));

  // set kp parameter
  p = task->lookupParameter("kp");
  EXPECT_TRUE(p) << "Unable to get kp parameter.";
  Vector kp(2); kp.setZero();
  EXPECT_TRUE(p->set(kp));

  // set kd parameter
  p = task->lookupParameter("kd");
  EXPECT_TRUE(p) << "Unable to get kd parameter.";
  Vector kd(2); kd << 1.0, 1.0;
  EXPECT_TRUE(p->set(kd));

  // set the maxVelocity parameter
  p = task->lookupParameter("maxVelocity");
  EXPECT_TRUE(p) << "Unable to get maxVelocity parameter.";
  Vector maxVelocity(2); maxVelocity.setZero();
  EXPECT_TRUE(p->set(maxVelocity));

  // set frameName parameter
  p = task->lookupParameter("frameName");
  EXPECT_TRUE(p) << "Unable to get frameName parameter.";
  EXPECT_TRUE(p->set("body1"));

  // set projection
  p = task->lookupParameter("projection");
  EXPECT_TRUE(p) << "Unable to get projection parameter.";
  Matrix projection(3,3); projection.setIdentity();
  projection(2,2) = 0; //COM2dTask!
  EXPECT_TRUE(p->set(projection));
  
  EXPECT_TRUE(task->init(*myModel)));

  // Update the inactive state within the task
  task->updateState(myModel.get());

  // Now that the inactive state is updated, make it the active state.
  task->checkUpdatedState();

  // Get the task's Jacobian
  Matrix Jtask;
  task->getJacobian(Jtask);
  ASSERT_EQ(Jtask.rows(), 3)
    << "Jtask has incorrect number of rows: expected 3 got "
    << Jtask.rows();
  ASSERT_EQ(Jtask.cols(), myModel->getNumDOFs())
    << "Task jacobian has incorrect number of columns: expected "
    << myModel->getNumDOFs() << " got " << Jtask.cols();

  TaskCommand command;
  task->getCommand(*myModel, command);
  EXPECT_TRUE(command.command.size() == 3) << "Error wrong command size. It was " << command.command.size();
  //EXPECT_TRUE(command.type == CommandType::ACCELERATION);
}

} // namespace task_library
} // namespace controlit
