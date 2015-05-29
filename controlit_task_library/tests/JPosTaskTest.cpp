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
#include <controlit/task_library/JPosTask.hpp>
#include <controlit/ControlModelLibrary.hpp>
#include <controlit/RobotState.hpp>

namespace controlit {
namespace task_library {

class JPosTaskTest : public ::testing::Test
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

TEST_F(JPosTaskTest, BasicTest)
{
  std::unique_ptr<controlit::Task> task(new controlit::task_library::JPosTask);

  int NactuatedJoints = myModel->getNActuableDOFs();
  ASSERT_EQ(NactuatedJoints, 2) << "Incorrect number of actuated joints: Got "
    << NactuatedJoints << " expected 2";

  // Set the goal position parameter
  p = task->lookupParameter("goalPosition");
  EXPECT_TRUE(p) << "Unable to get goalPosition parameter.";
  Vector goalPos(myModel->getNActuableDOFs()); goalPos.setZero();  // For a CartPosTask, the goal position as 3 dimensions: x, y, z
  EXPECT_TRUE(p->set(goalPos));

  // Set the goal velocity parameter
  p = task->lookupParameter("goalVelocity");
  EXPECT_TRUE(p) << "Unable to get goalVelocity parameter.";
  Vector goalVel(myModel->getNActuableDOFs()); goalVel.setZero();  // For a CartPosTask, the goal position as 3 dimensions: x, y, z
  EXPECT_TRUE(p->set(goalVel));

  // set kp parameter
  p = task->lookupParameter("kp");
  EXPECT_TRUE(p) << "Unable to get kp parameter.";
  double kp(10.0); //kp.setZero();
  EXPECT_TRUE(p->set(kp));

  // set kd parameter
  p = task->lookupParameter("kd");
  EXPECT_TRUE(p) << "Unable to get kd parameter.";
  double kd(40.0); //kd.setZero();
  EXPECT_TRUE(p->set(kd));

  // set the maxVelocity parameter
  p = task->lookupParameter("maxVelocity");
  EXPECT_TRUE(p) << "Unable to get maxVelocity parameter.";
  double maxVelocity(1.0); // maxVelocity.setZero();
  EXPECT_TRUE(p->set(maxVelocity));

  // Init task
  EXPECT_TRUE(task->init(*myModel)) << "Problems initializing task";

  // Update the inactive state within the task
  task->updateState(myModel.get());

  // Now that the inactive state is updated, make it the active state.
  task->checkUpdatedState();

  TaskCommand command;
  task->getCommand(*myModel, command);
  EXPECT_TRUE(command.command.size() == 2) << "Error wrong command size. It was " << command.command.size();;
  //EXPECT_TRUE(command.type == CommandType::ACCELERATION);
  //cout << "Command is : \n" << command.command << endl;

  Matrix Jtask;
  task->getJacobian(Jtask);

  Matrix U;
  myModel->constraints().getU(U);
  EXPECT_TRUE(Jtask == U);
}

} // namespace task_library
} // namespace controlit
