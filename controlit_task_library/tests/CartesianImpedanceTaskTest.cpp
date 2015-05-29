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
#include <controlit/task_library/CartesianImpedanceTask.hpp>
#include <controlit/ControlModelLibrary.hpp>
#include <controlit/RobotState.hpp>

namespace controlit {
namespace task_library {

class CartesianImpedanceTaskTest : public ::testing::Test
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

TEST_F(CartesianImpedanceTaskTest, BasicTest)
{
  std::unique_ptr<controlit::Task> task(new controlit::task_library::CartesianImpedanceTask);

  int NactuatedJoints = myModel->getNActuableDOFs();
  ASSERT_EQ(NactuatedJoints, 2) << "Incorrect number of actuated joints: Got "
    << NactuatedJoints << " expected 2";

  // Set the goal position parameter
  p = task->lookupParameter("goalPosition");
  EXPECT_TRUE(p) << "Unable to get goalPosition parameter.";
  Vector goalPos(7); goalPos.setZero();  // For a CartesianImpedanceTask, the goal position has 7 dimensions: {x,Q}
  EXPECT_TRUE(p->set(goalPos));

  // Set the goal velocity parameter
  p = task->lookupParameter("goalVelocity");
  EXPECT_TRUE(p) << "Unable to get goalPosition parameter.";
  Vector goalVel(6); goalVel.setZero();  // For a CartesianImpedanceTask, the goal velocity has 6 dimensions: {v,\omega}
  EXPECT_TRUE(p->set(goalVel));

  // Set the goal acceleration parameter
  p = task->lookupParameter("goalAcceleration");
  EXPECT_TRUE(p) << "Unable to get goalPosition parameter.";
  Vector goalAcc(6); goalAcc.setZero();  // For a CartesianImpedanceTask, the goal acceleration has 6 dimensions: {a,\alpha}
  EXPECT_TRUE(p->set(goalAcc));

  // Set the goal wrench parameter
  p = task->lookupParameter("goalWrench");
  EXPECT_TRUE(p) << "Unable to get goalPosition parameter.";
  Vector goalForce(6); goalForce.setZero();  // For a CartesianImpedanceTask, the goal wrench has 6 dimensions: {F,\Tau}
  EXPECT_TRUE(p->set(goalForce));

  // Set the actual wrench parameter (this will be sensor data)
  p = task->lookupParameter("measuredWrench");
  EXPECT_TRUE(p) << "Unable to get measuredWrench parameter.";
  Vector measuredWrench(6); measuredWrench.setZero();  // For a CartesianImpedanceTask, the measuredWrench has 6 dimensions: {F,\Tau}
  EXPECT_TRUE(p->set(measuredWrench));

  // set stiffness parameter
  p = task->lookupParameter("stiffness");
  EXPECT_TRUE(p) << "Unable to get stiffness parameter.";
  Vector stiffness(6); stiffness.setOnes(); //stiffness.setZero();
  EXPECT_TRUE(p->set(stiffness);

  // set damping parameter
  p = task->lookupParameter("damping");
  EXPECT_TRUE(p) << "Unable to get damping parameter.";
  Vector damping(6); damping.setOnes(); //damping.setZero();
  EXPECT_TRUE(p->set(damping));

  // set inertia parameter
  p = task->lookupParameter("inertia");
  EXPECT_TRUE(p) << "Unable to get inertia parameter.";
  Vector inertia(6); inertia.setOnes(); //inertia.setZero();
  EXPECT_TRUE(p->set(inertia);

  // set maxForce parameter
  p = task->lookupParameter("maxForce");
  EXPECT_TRUE(p) << "Unable to get maxForce parameter.";
  double maxForce = 1;
  EXPECT_TRUE(p->set(maxForce));

  // set maxTorque parameter
  p = task->lookupParameter("maxTorque");
  EXPECT_TRUE(p) << "Unable to get maxTorque parameter.";
  double maxTorque = 1;
  EXPECT_TRUE(p->set(maxTorque));

  // set Force sensor frame id
  p = task->lookupParameter("sensorFrame");
  EXPECT_TRUE(p) << "Unable to get sensorFrame parameter.";
  int sensorFrame = 0;
  EXPECT_TRUE(p->set(sensorFrame));

  // set bodyName parameter
  p = task->lookupParameter("bodyName");
  EXPECT_TRUE(p) << "Unable to get bodyName parameter.";
  EXPECT_TRUE(p->set("body3");

  // set frameName parameter
  p = task->lookupParameter("frameName");
  EXPECT_TRUE(p) << "Unable to get frameName parameter.";
  EXPECT_TRUE(p->set("body1"));

  // set controlPoint parameter
  p = task->lookupParameter("controlPoint");
  EXPECT_TRUE(p) << "Unable to get controlPoint parameter.";
  Vector controlPoint(7); controlPoint.setZero(); controlPoint(3) = 1.0; // control point is defined by {x,q}
  EXPECT_TRUE(p->set(controlPoint));

  // projection
  p = task->lookupParameter("projection");
  EXPECT_TRUE(p) << "Unable to get projection parameter.";
  Matrix projection(6,6); projection.setIdentity();
  EXPECT_TRUE(p->set(projection));

  EXPECT_TRUE(task->init(*myModel));

  // Get the task's Jacobian
  Matrix Jtask;
  task->getJacobian(Jtask);
  ASSERT_EQ(Jtask.rows(), 6)
    << "Jtask has incorrect number of rows: expected 6 got "
    << Jtask.rows();
  ASSERT_EQ(Jtask.cols(), myModel->getNumDOFs())
    << "Task jacobian has incorrect number of columns: expected "
    << myModel->getNumDOFs() << " got " << Jtask.cols();

  std::cout<<"Jtask = \n"<<Jtask<<"\n";

  TaskCommand command;
  task->getCommand(*myModel, command);
  EXPECT_TRUE(command.command.size() == 6) << "Error wrong command size. It was " << command.command.size();
  //EXPECT_TRUE(command.type == CommandType::ACCELERATION);

  std::cout << "command = \n" << command.command.transpose() << std::endl;
}

TEST_F(CartesianImpedanceTaskTest, WorldFrameTest)
{
  std::unique_ptr<controlit::Task> task(new controlit::task_library::CartesianImpedanceTask);

  int NactuatedJoints = myModel->getNActuableDOFs();
  ASSERT_EQ(NactuatedJoints, 2) << "Incorrect number of actuated joints: Got "
    << NactuatedJoints << " expected 2";

  // Set the goal position parameter
  p = task->lookupParameter("goalPosition");
  EXPECT_TRUE(p) << "Unable to get goalPosition parameter.";
  Vector goalPos(7); goalPos.setZero();  // For a CartesianImpedanceTask, the goal position has 7 dimensions: {x,Q}
  EXPECT_TRUE(p->set(goalPos));

  // Set the goal velocity parameter
  p = task->lookupParameter("goalVelocity");
  EXPECT_TRUE(p) << "Unable to get goalPosition parameter.";
  Vector goalVel(6); goalVel.setZero();  // For a CartesianImpedanceTask, the goal velocity has 6 dimensions: {v,\omega}
  EXPECT_TRUE(p->set(goalVel));

  // Set the goal acceleration parameter
  p = task->lookupParameter("goalAcceleration");
  EXPECT_TRUE(p) << "Unable to get goalPosition parameter.";
  Vector goalAcc(6); goalAcc.setZero();  // For a CartesianImpedanceTask, the goal acceleration has 6 dimensions: {a,\alpha}
  EXPECT_TRUE(p->set(goalAcc));

  // Set the goal wrench parameter
  p = task->lookupParameter("goalWrench");
  EXPECT_TRUE(p) << "Unable to get goalPosition parameter.";
  Vector goalForce(6); goalForce.setZero();  // For a CartesianImpedanceTask, the goal wrench has 6 dimensions: {F,\Tau}
  EXPECT_TRUE(p->set(goalForce));

  // Set the actual wrench parameter (this will be sensor data)
  p = task->lookupParameter("measuredWrench");
  EXPECT_TRUE(p) << "Unable to get measuredWrench parameter.";
  Vector measuredWrench(6); measuredWrench.setZero();  // For a CartesianImpedanceTask, the measuredWrench has 6 dimensions: {F,\Tau}
  EXPECT_TRUE(p->set(measuredWrench));

  // set stiffness parameter
  p = task->lookupParameter("stiffness");
  EXPECT_TRUE(p) << "Unable to get stiffness parameter.";
  Vector stiffness(6); stiffness.setOnes(); //stiffness.setZero();
  EXPECT_TRUE(p->set(stiffness));

  // set damping parameter
  p = task->lookupParameter("damping");
  EXPECT_TRUE(p) << "Unable to get damping parameter.";
  Vector damping(6); damping.setOnes(); //damping.setZero();
  EXPECT_TRUE(p->set(damping));

  // set inertia parameter
  p = task->lookupParameter("inertia");
  EXPECT_TRUE(p) << "Unable to get inertia parameter.";
  Vector inertia(6); inertia.setOnes(); //inertia.setZero();
  EXPECT_TRUE(p->set(inertia));

  // set maxForce parameter
  p = task->lookupParameter("maxForce");
  EXPECT_TRUE(p) << "Unable to get maxForce parameter.";
  double maxForce = 1;
  EXPECT_TRUE(p->set(maxForce));

  // set maxTorque parameter
  p = task->lookupParameter("maxTorque");
  EXPECT_TRUE(p) << "Unable to get maxTorque parameter.";
  double maxTorque = 1;
  EXPECT_TRUE(p->set(maxTorque));

  // set Force sensor frame id
  p = task->lookupParameter("sensorFrame");
  EXPECT_TRUE(p) << "Unable to get sensorFrame parameter.";
  int sensorFrame = 0;
  EXPECT_TRUE(p->set(sensorFrame));

  // set bodyName parameter
  p = task->lookupParameter("bodyName");
  EXPECT_TRUE(p) << "Unable to get bodyName parameter.";
  EXPECT_TRUE(p->set("body3"));

  // set frameName parameter
  p = task->lookupParameter("frameName");
  EXPECT_TRUE(p) << "Unable to get frameName parameter.";
  EXPECT_TRUE(p->set("world"));

  // set controlPoint parameter
  p = task->lookupParameter("controlPoint");
  EXPECT_TRUE(p) << "Unable to get controlPoint parameter.";
  Vector controlPoint(7); controlPoint.setZero(); controlPoint(3) = 1.0; // control point is defined by {x,q}
  EXPECT_TRUE(p->set(controlPoint));

  // projection
  p = task->lookupParameter("projection");
  EXPECT_TRUE(p) << "Unable to get projection parameter.";
  Matrix projection(6,6); projection.setIdentity();
  EXPECT_TRUE(p->set(projection));

  EXPECT_TRUE(task->init(*myModel));

  // Update the inactive state within the task
  task->updateState(myModel.get());

  // Now that the inactive state is updated, make it the active state.
  task->checkUpdatedState();

  // Get the task's Jacobian
  Matrix Jtask;
  task->getJacobian(Jtask);
  ASSERT_EQ(Jtask.rows(), 6)
      << "Jtask has incorrect number of rows: expected 6 got "
      << Jtask.rows();
  ASSERT_EQ(Jtask.cols(), myModel->getNumDOFs())
      << "Task jacobian has incorrect number of columns: expected "
      << myModel->getNumDOFs() << " got " << Jtask.cols();

  std::cout << "Jtask = \n" << Jtask << std::endl;

  TaskCommand command;
  task->getCommand(*myModel, command);
  EXPECT_TRUE(command.command.size() == 6) << "Error wrong command size. It was " << command.command.size();
  //EXPECT_TRUE(command.type == CommandType::ACCELERATION);

  std::cout << "command = \n" << command.command.transpose() << std::endl;
}

} // namespace task_library
} // namespace controlit
