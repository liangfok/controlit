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

#include <controlit/TaskCommand.hpp>
#include <controlit/ParameterReflection.hpp>
#include <controlit/task_library/JPosTask.hpp>
#include <controlit/task_library/PDController.hpp>

namespace controlit {
namespace task_library {

template<class TestData>
void test_controller(SaturationPolicy::Options policy, TestData& testData)
{
  // CONTROLIT_DEBUG_RT << "Method called with policy = " << policy;

  ros::Time::init();

  // Create a ParameterReflection object
  std::unique_ptr<controlit::ParameterReflection> pr;
  pr.reset(new controlit::ParameterReflection("__NO_TYPE__", "__NO_NAME__"));

  // Create the PDTask using the Norm saturation policy
  std::unique_ptr<PDController> controller(PDControllerFactory::create(policy));
  EXPECT_TRUE( controller.get() != NULL );
  controller->resize(2);
  controller->declareParameters(pr.get());

  for (auto & testcase : testData.testCases)
  {
    // Unpack the test case
    typename TestData::Input input;
    typename TestData::ExpectedResult result;
    std::tie(input, result) = testcase;

    // Unpack the input
    typename TestData::kp kp;
    typename TestData::Xd xd;
    typename TestData::X x;
    typename TestData::kd kd;
    typename TestData::Xd_dot xd_dot;
    typename TestData::X_dot x_dot;
    typename TestData::maxVelocity maxVelocity;
    typename TestData::maxAcceleration maxAcceleration;
    std::tie(kp, xd, x, kd, xd_dot, x_dot, maxVelocity, maxAcceleration) = input;

    // Set kp
    controlit::Parameter* p = pr->lookupParameter("kp");
    EXPECT_TRUE(p != NULL) << "Failed to lookup parameter kp";
    EXPECT_TRUE(p->set(kp)) << "Failed to set kp!";

    // Set kd
    p = pr->lookupParameter("kd");
    EXPECT_TRUE(p != NULL) << "Failed to lookup parameter kd";
    EXPECT_TRUE(p->set(kd)) << "Failed to set kd!";

    // Set maxVelocity.. if it has it
    p = pr->lookupParameter("maxVelocity");
    EXPECT_TRUE(p != NULL)  << "Failed to lookup parameter maxVelocity";
    EXPECT_TRUE(p->set(maxVelocity)) << "Failed to set maxVelocity!";

    // Set maxVelocity.. if it has it
    p = pr->lookupParameter("maxAcceleration");
    EXPECT_TRUE(p != NULL) << "Failed to lookup parameter maxAcceleration";
    EXPECT_TRUE(p->set(maxAcceleration)) << "Failed to set maxAcceleration!";

    // Compute output
    Vector cmd(2);
    controller->computeCommand(xd, x, xd_dot, x_dot, cmd, pr.get());

    // Unpack the results
    typename TestData::error error;
    typename TestData::error_dot error_dot;
    typename TestData::U u;

    std::tie(error, error_dot, u) = result;

    // Check results

    // CONTROLIT_DEBUG_RT << "Checking the results...";

    EXPECT_TRUE(cmd == u) << "Computed: " << cmd.transpose() << ", Expected: " << u.transpose();

    // Check other values

    p = pr->lookupParameter("error");
    EXPECT_TRUE(p != NULL) << "Failed to lookup parameter error";

    // CONTROLIT_DEBUG_RT << "Checking the error..., got = " << *(p->getVector()) << ", expected = " << error;

    EXPECT_TRUE(error == *(p->getVector())) << "Computed: " << *(p->getVector()) << ", Expected: " << error;

    // CONTROLIT_DEBUG_RT << "Checking the errorDot...";

    p = pr->lookupParameter("errorDot");
    EXPECT_TRUE(p != NULL) << "Failed to lookup parameter errorDot";

    // CONTROLIT_DEBUG_RT << "error_dot = " << error_dot.transpose() << ", p->getVector() = " << *(p->getVector());

    EXPECT_TRUE(error_dot == *(p->getVector())) << "Computed: " << *(p->getVector()) << ", Expected: " << error_dot;
  }
}

/*****************************************************************************
 * Policy: OFF
 *****************************************************************************/
struct NoPolicyTestData
{
  // Make test cases a little more readable
  typedef Vector kp;
  typedef Vector Xd;
  typedef Vector X;
  typedef Vector kd;
  typedef Vector Xd_dot;
  typedef Vector X_dot;
  typedef Vector maxVelocity;
  typedef Vector maxAcceleration;
  typedef Vector error;
  typedef Vector error_dot;
  typedef Vector U;

  typedef std::tuple<kp, Xd, X, kd, Xd_dot, X_dot, maxVelocity, maxAcceleration> Input;
  typedef std::tuple<error, error_dot, U> ExpectedResult;
  typedef std::tuple<Input, ExpectedResult> TestCase;
  typedef std::vector<TestCase> TestCases;
  TestCases testCases;

  NoPolicyTestData()
  {
    // CONTROLIT_DEBUG_RT << "Method Called!";

    // CONTROLIT_DEBUG_RT << "Saturation Policies:\n"
    //   << " - Off = " << SaturationPolicy::Off << "\n"
    //   << " - ComponentWiseVel = " << SaturationPolicy::ComponentWiseVel << "\n"
    //   << " - MaxComponentVel = " << SaturationPolicy::MaxComponentVel << "\n"
    //   << " - NormVel = " << SaturationPolicy::NormVel << "\n"
    //   << " - ComponentWiseAcc = " << SaturationPolicy::ComponentWiseAcc << "\n"
    //   << " - MaxComponentAcc = " << SaturationPolicy::MaxComponentAcc << "\n"
    //   << " - NormAcc = " << SaturationPolicy::NormAcc;

    // Declared test cases

    testCases.push_back( std::make_tuple(
      // // Input: kp, xd, x, kd, xd_dot, x_dot, max velocity, max acceleration
      std::make_tuple(kp::Ones(2), Xd::Zero(2), X::Zero(2), kd::Ones(2), Xd_dot::Zero(2), X_dot::Zero(2), maxVelocity::Zero(2), maxAcceleration::Zero(2)),
      // ExpectedResult: error, e, error_dot, e_dot, U
      std::make_tuple(error::Zero(2), error_dot::Zero(2), U::Zero(2))
    ));

    testCases.push_back( std::make_tuple(
      // Input: kp, xd, x, kd, xd_dot, x_dot, max velocity, max acceleration
      std::make_tuple(kp::Zero(2), Xd::Ones(2), X::Zero(2), kd::Zero(2), Xd_dot::Ones(2), X_dot::Zero(2), maxVelocity::Zero(2), maxAcceleration::Zero(2)),
      // ExpectedResult: error, e, error_dot, e_dot, U
      std::make_tuple(error::Ones(2), error_dot::Ones(2), U::Zero(2))
    ));

    testCases.push_back( std::make_tuple(
      // Input: kp, xd, x, kd, xd_dot, x_dot, max velocity, max acceleration
      std::make_tuple(kp::Ones(2), Xd::Ones(2), X::Zero(2), kd::Ones(2), Xd_dot::Ones(2), X_dot::Zero(2), maxVelocity::Zero(2), maxAcceleration::Zero(2)),
      // ExpectedResult: error, e, error_dot, e_dot, U
      std::make_tuple(error::Ones(2), error_dot::Ones(2), U::Ones(2) * 2)
    ));

    testCases.push_back( std::make_tuple(
      // Input: kp, xd, x, kd, xd_dot, x_dot, max velocity, max acceleration
      std::make_tuple(kp::Ones(2), Xd::Zero(2), X::Ones(2), kd::Ones(2), Xd_dot::Zero(2), X_dot::Ones(2), maxVelocity::Zero(2), maxAcceleration::Zero(2)),
      // ExpectedResult: error, e, error_dot, e_dot, U
      std::make_tuple(error::Ones(2) * -1, error_dot::Ones(2) * -1, U::Ones(2) * -2)
    ));
  };
};

TEST(PDControllerTest, NoPolicy)
{
  NoPolicyTestData testData;
  test_controller(SaturationPolicy::Off, testData);
}


/*****************************************************************************
 * Policy: ComponentWise
 *****************************************************************************/
struct ComponentWiseTestData
{
 // Make test cases a little more readable
 typedef Vector kp;
 typedef Vector Xd;
 typedef Vector X;
 typedef Vector kd;
 typedef Vector Xd_dot;
 typedef Vector X_dot;
 typedef Vector maxVelocity;
 typedef Vector maxAcceleration;
 typedef Vector error;
 typedef Vector error_dot;
 typedef Vector U;

 typedef std::tuple<kp, Xd, X, kd, Xd_dot, X_dot, maxVelocity, maxAcceleration> Input;
 typedef std::tuple<error, error_dot, U> ExpectedResult;
 typedef std::tuple<Input, ExpectedResult> TestCase;
 typedef std::vector<TestCase> TestCases;
 TestCases testCases;

 ComponentWiseTestData()
 {
   // Declared test cases
   testCases.push_back( std::make_tuple(
     // Input: kp, xd, x, kd, xd_dot, x_dot, max velocity, max acceleration
     std::make_tuple(kp::Ones(2), Xd::Zero(2), X::Zero(2), kd::Ones(2), Xd_dot::Zero(2), X_dot::Zero(2), maxVelocity::Ones(2)*10, maxAcceleration::Ones(2)*10),
     // ExpectedResult: error, e, error_dot, e_dot, U
     std::make_tuple(error::Zero(2), error_dot::Zero(2), U::Zero(2))
   ));

   testCases.push_back( std::make_tuple(
     // Input: kp, xd, x, kd, xd_dot, x_dot, max velocity, max acceleration
     std::make_tuple(kp::Zero(2), Xd::Ones(2), X::Zero(2), kd::Zero(2), Xd_dot::Ones(2), X_dot::Zero(2), maxVelocity::Ones(2)*10, maxAcceleration::Ones(2)*10),
     // ExpectedResult: error, e, error_dot, e_dot, U
     std::make_tuple(error::Ones(2), error_dot::Ones(2), U::Zero(2))
   ));

   testCases.push_back( std::make_tuple(
     // Input: kp, xd, x, kd, xd_dot, x_dot, max velocity, max acceleration
     std::make_tuple(kp::Ones(2), Xd::Ones(2), X::Zero(2), kd::Ones(2), Xd_dot::Ones(2), X_dot::Zero(2), maxVelocity::Ones(2)*10, maxAcceleration::Ones(2)*10),
     // ExpectedResult: error, e, error_dot, e_dot, U
     std::make_tuple(error::Ones(2), error_dot::Ones(2), U::Ones(2) * 2)
   ));

   testCases.push_back( std::make_tuple(
     // Input: kp, xd, x, kd, xd_dot, x_dot, max velocity, max acceleration
     std::make_tuple(kp::Ones(2), Xd::Zero(2), X::Ones(2), kd::Ones(2), Xd_dot::Zero(2), X_dot::Ones(2), maxVelocity::Ones(2)*10, maxAcceleration::Ones(2)*10),
     // ExpectedResult: error, e, error_dot, e_dot, U
     std::make_tuple(error::Ones(2) * -1, error_dot::Ones(2) * -1, U::Ones(2) * -2)
   ));

   // TODO: Add saturation-specific policy tests here!
 };
};

TEST(PDControllerTest, ComponentWiseVel)
{
  ComponentWiseTestData testData;
  test_controller(SaturationPolicy::ComponentWiseVel, testData);
}

/*****************************************************************************
 * Policy: MaxComponent
 *****************************************************************************/
struct MaxComponentTestData
{
 // Make test cases a little more readable
 typedef Vector kp;
 typedef Vector Xd;
 typedef Vector X;
 typedef Vector kd;
 typedef Vector Xd_dot;
 typedef Vector X_dot;
 typedef Vector maxVelocity;
 typedef Vector maxAcceleration;
 typedef Vector error;
 typedef Vector error_dot;
 typedef Vector U;

 typedef std::tuple<kp, Xd, X, kd, Xd_dot, X_dot, maxVelocity, maxAcceleration> Input;
 typedef std::tuple<error, error_dot, U> ExpectedResult;
 typedef std::tuple<Input, ExpectedResult> TestCase;
 typedef std::vector<TestCase> TestCases;
 TestCases testCases;

 MaxComponentTestData()
 {
   // Declared test cases
   testCases.push_back( std::make_tuple(
     // Input: kp, xd, x, kd, xd_dot, x_dot, max velocity, max acceleration
     std::make_tuple(kp::Ones(2), Xd::Zero(2), X::Zero(2), kd::Ones(2), Xd_dot::Zero(2), X_dot::Zero(2), maxVelocity::Ones(2)*10, maxAcceleration::Ones(2)*10),
     // ExpectedResult: error, e, error_dot, e_dot, U
     std::make_tuple(error::Zero(2), error_dot::Zero(2), U::Zero(2))
   ));

   testCases.push_back( std::make_tuple(
     // Input: kp, xd, x, kd, xd_dot, x_dot, max velocity, max acceleration
     std::make_tuple(kp::Zero(2), Xd::Ones(2), X::Zero(2), kd::Zero(2), Xd_dot::Ones(2), X_dot::Zero(2), maxVelocity::Ones(2)*10, maxAcceleration::Ones(2)*10),
     // ExpectedResult: error, e, error_dot, e_dot, U
     std::make_tuple(error::Ones(2), error_dot::Ones(2), U::Zero(2))
   ));

   testCases.push_back( std::make_tuple(
     // Input: kp, xd, x, kd, xd_dot, x_dot, max velocity, max acceleration
     std::make_tuple(kp::Ones(2), Xd::Ones(2), X::Zero(2), kd::Ones(2), Xd_dot::Ones(2), X_dot::Zero(2), maxVelocity::Ones(2)*10, maxAcceleration::Ones(2)*10),
     // ExpectedResult: error, e, error_dot, e_dot, U
     std::make_tuple(error::Ones(2), error_dot::Ones(2), U::Ones(2) * 2)
   ));

   testCases.push_back( std::make_tuple(
     // Input: kp, xd, x, kd, xd_dot, x_dot, max velocity, max acceleration
     std::make_tuple(kp::Ones(2), Xd::Zero(2), X::Ones(2), kd::Ones(2), Xd_dot::Zero(2), X_dot::Ones(2), maxVelocity::Ones(2)*10, maxAcceleration::Ones(2)*10),
     // ExpectedResult: error, e, error_dot, e_dot, U
     std::make_tuple(error::Ones(2) * -1, error_dot::Ones(2) * -1, U::Ones(2) * -2)
   ));

   // TODO: Add staturation-specific policy tests here!
 };
};

TEST(PDControllerTest, MaxComponentVel)
{
 MaxComponentTestData testData;
 test_controller(SaturationPolicy::MaxComponentVel, testData);
}


/*****************************************************************************
 Policy: Norm
*****************************************************************************/
struct NormTestData
{
 // Make test cases a little more readable
 typedef double kp;
 typedef Vector Xd;
 typedef Vector X;
 typedef double kd;
 typedef Vector Xd_dot;
 typedef Vector X_dot;
 typedef double maxVelocity;
 typedef double maxAcceleration;
 typedef Vector error;
 typedef Vector error_dot;
 typedef Vector U;

 typedef std::tuple<kp, Xd, X, kd, Xd_dot, X_dot, maxVelocity, maxAcceleration> Input;
 typedef std::tuple<error, error_dot, U> ExpectedResult;
 typedef std::tuple<Input, ExpectedResult> TestCase;
 typedef std::vector<TestCase> TestCases;
 TestCases testCases;

 NormTestData()
 {
   // Declared test cases
   testCases.push_back( std::make_tuple(
     // Input: kp, xd, x, kd, xd_dot, x_dot, max velocity, max acceleration
     std::make_tuple(kp(1.0), Xd::Zero(2), X::Zero(2), kd(1.0), Xd_dot::Zero(2), X_dot::Zero(2), maxVelocity(10.0), maxAcceleration(10.0)),
     // ExpectedResult: error, e, error_dot, e_dot, U
     std::make_tuple(error::Zero(2), error_dot::Zero(2), U::Zero(2))
   ));

   testCases.push_back( std::make_tuple(
     // Input: kp, xd, x, kd, xd_dot, x_dot, max velocity, max acceleration
     std::make_tuple(kp(0.0), Xd::Ones(2), X::Zero(2), kd(0.0), Xd_dot::Ones(2), X_dot::Zero(2), maxVelocity(10.0), maxAcceleration(10.0)),
     // ExpectedResult: error, e, error_dot, e_dot, U
     std::make_tuple(error::Ones(2), error_dot::Ones(2), U::Zero(2))
   ));

   testCases.push_back( std::make_tuple(
     // Input: kp, xd, x, kd, xd_dot, x_dot, max velocity, max acceleration
     std::make_tuple(kp(1.0), Xd::Ones(2), X::Zero(2), kd(1.0), Xd_dot::Ones(2), X_dot::Zero(2), maxVelocity(10.0), maxAcceleration(10.0)),
     // ExpectedResult: error, e, error_dot, e_dot, U
     std::make_tuple(error::Ones(2), error_dot::Ones(2), U::Ones(2) * 2)
   ));

   testCases.push_back( std::make_tuple(
     // Input: kp, xd, x, kd, xd_dot, x_dot, max velocity, max acceleration
     std::make_tuple(kp(1.0), Xd::Zero(2), X::Ones(2), kd(1.0), Xd_dot::Zero(2), X_dot::Ones(2), maxVelocity(10.0), maxAcceleration(10.0)),
     // ExpectedResult: error, e, error_dot, e_dot, U
     std::make_tuple(error::Ones(2) * -1, error_dot::Ones(2) * -1, U::Ones(2) * -2)
   ));

   // TODO: Add staturation-specific policy tests here!
 };
};

TEST(PDControllerTest, NormVel)
{
  NormTestData testData;
  test_controller(SaturationPolicy::NormVel, testData);
}


//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
/*****************************************************************************
 * Policy: ComponentWiseAcc
 *****************************************************************************/

TEST(PDControllerTest, ComponentWiseAcc)
{
  ComponentWiseTestData testData;
  test_controller(SaturationPolicy::ComponentWiseAcc, testData);
}


/*****************************************************************************
 * Policy: MaxComponentVel
 *****************************************************************************/
TEST(PDControllerTest, MaxComponentAcc)
{
  MaxComponentTestData testData;
  test_controller(SaturationPolicy::MaxComponentAcc, testData);
}


/*****************************************************************************
 * Policy: NormVel
 *****************************************************************************/
TEST(PDControllerTest, NormAcc)
{
  NormTestData testData;
  test_controller(SaturationPolicy::NormAcc, testData);
}

} // namespace task_library
} // namespace controlit