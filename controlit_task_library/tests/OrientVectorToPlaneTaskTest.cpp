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
#include <controlit/task_library/OrientVectorToPlaneTask.hpp>
#include <controlit/ControlModelLibrary.hpp>
#include <controlit/RobotState.hpp>

namespace controlit {
namespace task_library {

class OrientVectorToPlaneTaskTest : public ::testing::Test
{
private:

  void createModel()
  {
    robotState.reset(new controlit::RobotState());
    controlit::ControlModel * controlModel = controlit::ControlModelLibrary::getLegAndFootModel(robotState);
    myModel.reset(controlModel);

    // Verify the number of actuable joints is as expected.
    int NactuatedJoints = myModel->getNActuableDOFs();
    ASSERT_EQ(NactuatedJoints, 2)
      << "Incorrect number of actuated joints: Got "
      << NactuatedJoints << " expected 2";
  }

  void createTask()
  {
    task.reset(new controlit::task_library::OrientVectorToPlaneTask());

    // Get parameter goalVector
    controlit::Parameter * p = task->lookupParameter("normalVector");
    EXPECT_TRUE(p) << "Unable to get parameter \"normalVector\".";

    // For an OrientVectorToVectorTask, the goalVector parameter has 3 dimensions: x, y, z.
    // Set parameter goalVector to be (1, 1, 0)
    Vector goalPos(3); goalPos << 0, 0, 1;
    s = p->set(goalPos);
    EXPECT_TRUE(s) << s.what();

    // Get parameter bodyFrameVector
    p = task->lookupParameter("bodyFrameVector");
    EXPECT_TRUE(p) << "Unable to get parameter \"bodyFrameVector\".";

    // For a OrientVectorToVectorTask, the bodyFrameVector parameter has 3 dimensions: x, y, z.
    // Set parameter bodyFrameVector to be (1, 0, 0)
    Vector bodyFrameVector(3); bodyFrameVector << 1, 0, 0;
    s = p->set(bodyFrameVector);
    EXPECT_TRUE(s) << s.what();

    // Get parameter kp
    p = task->lookupParameter("kp");
    EXPECT_TRUE(p) << "Unable to get parameter \"kp\".";

    // Set parameter kp to be 1
    double kp = 1;
    s = p->set(kp);
    EXPECT_TRUE(s) << s.what();

    // Get parameter kd
    p = task->lookupParameter("kd");
    EXPECT_TRUE(p) << "Unable to get parameter \"kd\".";

    // Set parameter kd = 1
    double kd = 1;
    s = p->set(kd);
    EXPECT_TRUE(s) << s.what();

    // Get parameter maxVelocity
    p = task->lookupParameter("maxVelocity");
    EXPECT_TRUE(p) << "Unable to get parameter \"maxVelocity\".";

    // Set parameter maxVelocity to be 0
    double maxVelocity = 0;
    s = p->set(maxVelocity);
    EXPECT_TRUE(s) << s.what();

    // Get parameter bodyName
    p = task->lookupParameter("bodyName");
    EXPECT_TRUE(p) << "Unable to get parameter \"bodyName\".";

    // Set parameter bodyName to be revolute1DoF_2
    s = p->set("revolute1DoF_2");
    EXPECT_TRUE(s) << s.what();
  }

protected:
  virtual void SetUp()
  {
    ros::Time::init();
    createModel();
    createTask();
  }

  virtual void TearDown()
  {
    myModel.reset();
  }

  std::shared_ptr<controlit::RobotState> robotState;
  std::unique_ptr<controlit::ControlModel> myModel;
  std::unique_ptr<controlit::Task> task;
  controlit::Parameter * p;
};

TEST_F(OrientVectorToPlaneTaskTest, WorldFrameTest)
{
  // Get parameter frameName
  p = task->lookupParameter("frameName");
  EXPECT_TRUE(p) << "Unable to get parameter \"frameName\".";

  // Set parameter frameName to be World
  s = p->set("world");
  EXPECT_TRUE(s) << s.what();

  s = task->init(*myModel) ;
  EXPECT_TRUE(s) << s.what();

  // Update the inactive state within the task
  task->updateState(myModel.get());

  // Now that the inactive state is updated, make it the active state.
  task->checkUpdatedState();

  // Get the task's Jacobian
  Matrix Jtask;
  task->getJacobian(Jtask);

  // Verify that the matrix has the right dimensions
  ASSERT_EQ(Jtask.rows(), 1)
    << "Jtask has incorrect number of rows: expected 1 got "
    << Jtask.rows();

  ASSERT_EQ(Jtask.cols(), myModel->getNumDOFs())
    << "Task jacobian has incorrect number of columns: expected "
    << myModel->getNumDOFs() << " got " << Jtask.cols();

  // Get the command
  TaskCommand command;
  s = task->getCommand(*myModel, command);
  EXPECT_TRUE(s) << s.what();

  EXPECT_TRUE(command.command.size() == 1)
    << "Unexpected command size.  Got " << command.command.size() << ", expected 1.";
  EXPECT_TRUE(command.type == CommandType::ACCELERATION)
    << "Unexpected command type.  Got " << command.type << ", expected " << CommandType::ACCELERATION;


  // std::cout << std::scientific << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 1)
 //    << "OrientVectorToPlaneTask's Jacobian is:\n" << Jtask << std::endl;

  // Verify that the task's Jacobian matrix is approximately correct
  Matrix expectedTaskJacobian(1, myModel->getNumDOFs());
  expectedTaskJacobian << 0, 0, 0, 0, -1, 0, 0, -1;

  double errorNorm = (expectedTaskJacobian - Jtask).norm();

  EXPECT_TRUE(errorNorm < 1e-10)
    << "OrientVectorToPlaneTask's Jacobian did not match expected value!\n"
    << "Got:\n"
    << Jtask << "\n"
    << "Expected:\n"
    << expectedTaskJacobian << "\n"
    << "Norm of difference = " << errorNorm;

}

TEST_F(OrientVectorToPlaneTaskTest, LatchedBodyFrameTest)
{
  // Get parameter frameName
  p = task->lookupParameter("frameName");
  EXPECT_TRUE(p) << "Unable to get parameter \"frameName\".";

  // Set parameter frameName to be "body1"
  s = p->set("body1");
  EXPECT_TRUE(s) << s.what();

  // Get parameter latchOn
  p = task->lookupParameter("latchOn");
  EXPECT_TRUE(p) << "Unable to get parameter \"latchOn\".";

  // Set parameter latchOn to be true (1)
  EXPECT_TRUE(p->set(1));
  EXPECT_TRUE(task->init(*myModel));

  // Get the task's command
  TaskCommand command;
  EXPECT_TRUE(task->getCommand(*myModel, command));

  EXPECT_TRUE(command.command.size() == 1)
    << "Unexpected command size.  Got " << command.command.size() << ", expected 1.";
  EXPECT_TRUE(command.type == CommandType::ACCELERATION)
    << "Unexpected command type.  Got " << command.type << ", expected " << CommandType::ACCELERATION;

  // Update the inactive state within the task
  task->updateState(myModel.get());

  // Now that the inactive state is updated, make it the active state.
  task->checkUpdatedState();

  // Get the task's Jacobian
  Matrix Jtask;
  task->getJacobian(Jtask);

  // Verify that the matrix has the right dimensions
  ASSERT_EQ(Jtask.rows(), 1)
    << "Jtask has incorrect number of rows: expected 1 got "
    << Jtask.rows();

  ASSERT_EQ(Jtask.cols(), myModel->getNumDOFs())
    << "Task jacobian has incorrect number of columns: expected "
    << myModel->getNumDOFs() << " got " << Jtask.cols();

  // std::cout << std::scientific << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 1)
 //    << "OrientVectorToPlaneTask's Jacobian is:\n" << Jtask << std::endl;

  // Verify that the task's Jacobian matrix is approximately correct
  Matrix expectedTaskJacobian(1, myModel->getNumDOFs());
  expectedTaskJacobian << 0, 0, 0, 0, -1, 0, 0, -1;

  double errorNorm = (expectedTaskJacobian - Jtask).norm();

  EXPECT_TRUE(errorNorm < 1e-10)
    << "OrientVectorToPlaneTask's Jacobian did not match expected value!\n"
    << "Got:\n"
    << Jtask << "\n"
    << "Expected:\n"
    << expectedTaskJacobian << "\n"
    << "Norm of difference = " << errorNorm;

}

} // namespace task_library
} // namespace controlit
