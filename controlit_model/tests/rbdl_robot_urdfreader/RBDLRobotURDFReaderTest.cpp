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
#include <controlit_robot_models/rbdl_robot_urdfreader.hpp>
#include <drc/logging/logging.hpp>

class RBDLRobotURDFReaderTest : public ::testing::Test
{
protected:
  virtual void SetUp() {}
  virtual void TearDown() {}
};

/* ----------------------------------------------------------------------------
 *  RBDLRobot functionality
 * --------------------------------------------------------------------------*/
TEST_F(RBDLRobotURDFReaderTest, RobotInitFromFileTestFloating)
{
  RigidBodyDynamics::Model * myRobot = new RigidBodyDynamics::Model();
  myRobot->Init();

  std::map<std::string, std::string> LinkNameToJointName;

  bool verbose = true;
  bool worked = controlit::rbdl_robot_urdfreader::read_urdf_model_from_file(
    "tests/rbdl_robot_urdfreader/wbc_minimal_robot_3dof.urdf",
    myRobot, &LinkNameToJointName, verbose);

  EXPECT_TRUE(worked) << "Problem while reading URDF!";

  //Robot testing
  EXPECT_TRUE(myRobot->GetBodyId("ROOT") == 0);
  EXPECT_TRUE(myRobot->GetBodyId("rigid6DoF") == 6);
  EXPECT_TRUE(myRobot->GetBodyId("revolute1DoF_1") == 7);
  EXPECT_TRUE(myRobot->GetBodyId("revolute1DoF_2") == 8);
  EXPECT_TRUE(myRobot->GetBodyId("revolute1DoF_3") == 9);

  //Link map testing
  EXPECT_TRUE(LinkNameToJointName["robot_body_1"] == "rigid6DoF");
  EXPECT_TRUE(LinkNameToJointName["robot_body_2"] == "revolute1DoF_1");
  EXPECT_TRUE(LinkNameToJointName["robot_body_3"] == "revolute1DoF_2");
  EXPECT_TRUE(LinkNameToJointName["robot_body_4"] == "revolute1DoF_3");

  std::map<std::string, std::string>::iterator it;
  for(it = LinkNameToJointName.begin(); it != LinkNameToJointName.end(); it++)
    std::cout << "LinkNameToJointName[ " << it->first << " ] = " << it->second << std::endl;

  delete myRobot;
}

TEST_F(RBDLRobotURDFReaderTest, RobotInitFromFileTestNoFloating)
{
  RigidBodyDynamics::Model * myRobot = new RigidBodyDynamics::Model();
  myRobot->Init();

  std::map<std::string, std::string> LinkNameToJointName;

  bool verbose = true;
  bool worked = controlit::rbdl_robot_urdfreader::read_urdf_model_from_file(
    "tests/rbdl_robot_urdfreader/wbc_minimal_robot_3dof_noFloating.urdf",
    myRobot, &LinkNameToJointName, verbose);

  EXPECT_TRUE(worked) << "Problem while reading URDF!";

  //Robot testing
  EXPECT_TRUE(myRobot->GetBodyId("ROOT") == 0);
  EXPECT_TRUE(myRobot->GetBodyId("rigid6DoF") == 6);
  EXPECT_TRUE(myRobot->GetBodyId("revolute1DoF_1") == 7);
  EXPECT_TRUE(myRobot->GetBodyId("revolute1DoF_2") == 8);
  EXPECT_TRUE(myRobot->GetBodyId("revolute1DoF_3") == 9);

  //Link map testing
  EXPECT_TRUE(LinkNameToJointName["robot_body_1"] == "rigid6DoF");
  EXPECT_TRUE(LinkNameToJointName["robot_body_2"] == "revolute1DoF_1");
  EXPECT_TRUE(LinkNameToJointName["robot_body_3"] == "revolute1DoF_2");
  EXPECT_TRUE(LinkNameToJointName["robot_body_4"] == "revolute1DoF_3");

  std::map<std::string, std::string>::iterator it;
  for(it = LinkNameToJointName.begin(); it != LinkNameToJointName.end(); it++)
    std::cout << "LinkNameToJointName[ " << it->first << " ] = " << it->second << std::endl;

  delete myRobot;
}
