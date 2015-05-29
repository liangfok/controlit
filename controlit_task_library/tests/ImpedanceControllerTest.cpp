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
#include <controlit/task_library/ImpedanceController.hpp>

namespace controlit {
namespace task_library {

template<class TestData>
void test_controller(ImpedancePolicy::Options policy, TestData& testData)
{
  // CONTROLIT_DEBUG_RT << "Method called with policy = " << policy;

  ros::Time::init();

  // Create a ParameterReflection object
  std::unique_ptr<controlit::ParameterReflection> pr;
  pr.reset(new controlit::ParameterReflection("__NO_TYPE__", "__NO_NAME__"));

  // Create the Impedance Task
  std::unique_ptr<ImpedanceController> controller(ImpedanceControllerFactory::create(policy));
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
    typename TestData::TestName testname;
    typename TestData::stiffness stiffness;
    typename TestData::damping damping;
    typename TestData::inertia inertia;
    typename TestData::Xd xd;
    typename TestData::X x;
    typename TestData::Xd_dot xd_dot;
    typename TestData::X_dot x_dot;
    typename TestData::Xd_dot_dot xd_dot_dot;
    typename TestData::Fd fd;
    typename TestData::F f;

    //NormStiffness and PiecewiseNormStiffness
    typename TestData::MaxForce maxForce;

    //PiecewiseNormStiffness
    typename TestData::MaxTorque maxTorque;
    typename TestData::NumForceComponents numForceComponents;

    std::tie(testname, stiffness, damping, inertia, maxForce, maxTorque, numForceComponents,
             xd, x, xd_dot, x_dot, xd_dot_dot, fd, f) = input;

    // Set stiffness
    controlit::Parameter* p = pr->lookupParameter("stiffness");
    EXPECT_TRUE(p != NULL) << testname + ": Failed to lookup parameter stiffness";
    EXPECT_TRUE(p->set(stiffness)) << testname + ": Failed to set stiffness!";

    // Set damping
    p = pr->lookupParameter("damping");
    EXPECT_TRUE(p != NULL) << testname + ": Failed to lookup parameter damping";
    EXPECT_TRUE(p->set(damping)) << testname + ": Failed to set damping!";

    // Set inertia
    p = pr->lookupParameter("inertia");
    EXPECT_TRUE(p != NULL)  << testname + ": Failed to lookup parameter inertia";
    EXPECT_TRUE(p->set(inertia)) << testname + ": Failed to set inertia!";

    if(policy == ImpedancePolicy::Options::NormStiffness ||
       policy == ImpedancePolicy::Options::PiecewiseNormStiffness)
    {
      // Set maxForce ... if it has it
      p = pr->lookupParameter("maxForce");
      EXPECT_TRUE(p != NULL) << testname + ": failed to lookup parameter maxForce";
      EXPECT_TRUE(p->set(maxForce)) << testname + ": failed to set maxForce!";

      if(policy == ImpedancePolicy::Options::PiecewiseNormStiffness)
      {
        // Set maxTorque ... if it has it
        p = pr->lookupParameter("maxTorque");
        EXPECT_TRUE(p != NULL) << testname + ": failed to lookup parameter maxTorque";
        EXPECT_TRUE(p->set(maxTorque)) << testname + ": failed to set maxTorque!";

        // Set numForceComponents ... if it has it
        p = pr->lookupParameter("numForceComponents");
        EXPECT_TRUE(p != NULL) << testname + ": failed to lookup parameter numForceComponents";
        EXPECT_TRUE(p->set(numForceComponents)) << testname + ": failed to set numForceComponents!";
      }
    }

    // Compute output
    Vector cmd(2);
    controller->resize(2);
    controller->computeCommand(xd, x, xd_dot, x_dot, xd_dot_dot, fd, f, cmd, pr.get());

    // Unpack the results
    typename TestData::error error;
    typename TestData::error_dot error_dot;
    typename TestData::U u;

    std::tie(error, error_dot, u) = result;

    // Check results

    // CONTROLIT_DEBUG_RT << "Checking the results...";

    EXPECT_TRUE(cmd == u) << testname + ": Computed: " << cmd.transpose() << ", Expected: " << u.transpose();

    // Check other values

    p = pr->lookupParameter("error");
    EXPECT_TRUE(p != NULL) << testname + ": Failed to lookup parameter error";

    // CONTROLIT_DEBUG_RT << "Checking the error..., got = " << *(p->getVector()) << ", expected = " << error;

    EXPECT_TRUE(error == *(p->getVector())) << testname + ": Computed: " << *(p->getVector()) << ", Expected: " << error;

    // CONTROLIT_DEBUG_RT << "Checking the errorDot...";

    p = pr->lookupParameter("errorDot");
    EXPECT_TRUE(p != NULL) << testname + ": Failed to lookup parameter errorDot";

    // CONTROLIT_DEBUG_RT << "error_dot = " << error_dot.transpose() << ", p->getVector() = " << *(p->getVector());

    EXPECT_TRUE(error_dot == *(p->getVector())) << testname + ": Computed: " << *(p->getVector()) << ", Expected: " << error_dot;
  }
}


/*****************************************************************************
 * Policy: PD
 *****************************************************************************/
struct GenericTestData
{
 // Make test cases a little more readable
 typedef std::string TestName;
 typedef Vector stiffness;
 typedef Vector damping;
 typedef Vector inertia;
 typedef Vector Xd;
 typedef Vector X;
 typedef Vector Xd_dot;
 typedef Vector X_dot;
 typedef Vector Xd_dot_dot;
 typedef Vector Fd;
 typedef Vector F;
 typedef Vector error;
 typedef Vector error_dot;
 typedef Vector U;

 typedef double MaxForce;
 typedef double MaxTorque;
 typedef int NumForceComponents;

 typedef std::tuple<TestName, stiffness, damping, inertia, MaxForce, MaxTorque, NumForceComponents,
                    Xd, X, Xd_dot, X_dot, Xd_dot_dot, Fd, F> Input;
 typedef std::tuple<error, error_dot, U> ExpectedResult;
 typedef std::tuple<Input, ExpectedResult> TestCase;
 typedef std::vector<TestCase> TestCases;
 TestCases testCases;
};

struct PDTestData : public GenericTestData
{

 //Generate PD specific test cases.
 PDTestData()
 {
   // Declared test cases
   testCases.push_back( std::make_tuple(
     // Input: stiffness, damping, inertia, xd, x, xd_dot, x_dot, xd_dot_dot, Fd, F
     std::make_tuple("Imedance PD1",
                     stiffness::Ones(2),
                     damping::Ones(2),
                     inertia::Ones(2),
                     0.0,
                     0.0,
                     0,
                     Xd::Zero(2),
                     X::Zero(2),
                     Xd_dot::Zero(2),
                     X_dot::Zero(2),
                     Xd_dot_dot::Zero(2),
                     Fd::Zero(2),
                     F::Zero(2)),
     // ExpectedResult: error, e, error_dot, U
     std::make_tuple(error::Zero(2), error_dot::Zero(2), U::Zero(2))
   ));

   // Declared test cases
   testCases.push_back( std::make_tuple(
     // Input: stiffness, damping, inertia, xd, x, xd_dot, x_dot, xd_dot_dot, Fd, F
     std::make_tuple("Imedance PD2",stiffness::Ones(2),
                     damping::Ones(2),
                     inertia::Ones(2),
                     0.0,
                     0.0,
                     0,
                     Xd::Zero(2),
                     X::Zero(2),
                     Xd_dot::Zero(2),
                     X_dot::Zero(2),
                     Xd_dot_dot::Zero(2),
                     Fd::Zero(2),
                     F::Ones(2)),
     // ExpectedResult: error, e, error_dot, U
     std::make_tuple(error::Zero(2), error_dot::Zero(2), U::Ones(2) * -1)
   ));

   // Declared test cases
   testCases.push_back( std::make_tuple(
     // Input: stiffness, damping, inertia, xd, x, xd_dot, x_dot, xd_dot_dot, Fd, F
     std::make_tuple("Imedance PD3",stiffness::Ones(2),
                     damping::Ones(2),
                     inertia::Ones(2) * 0.5,
                     0.0,
                     0.0,
                     0,
                     Xd::Zero(2),
                     X::Zero(2),
                     Xd_dot::Zero(2),
                     X_dot::Zero(2),
                     Xd_dot_dot::Zero(2),
                     Fd::Zero(2),
                     F::Ones(2)),
     // ExpectedResult: error, e, error_dot, U
     std::make_tuple(error::Zero(2), error_dot::Zero(2), U::Ones(2) * -2)
   ));

   // Declared test cases
   testCases.push_back( std::make_tuple(
     // Input: stiffness, damping, inertia, xd, x, xd_dot, x_dot, xd_dot_dot, Fd, F
     std::make_tuple("Imedance PD4",stiffness::Ones(2),
                     damping::Ones(2),
                     inertia::Ones(2),
                     0.0,
                     0.0,
                     0,
                     Xd::Ones(2),
                     X::Zero(2),
                     Xd_dot::Zero(2),
                     X_dot::Zero(2),
                     Xd_dot_dot::Zero(2),
                     Fd::Zero(2),
                     F::Ones(2)),
     // ExpectedResult: error, e, error_dot, U
     std::make_tuple(error::Ones(2), error_dot::Zero(2), U::Zero(2))
   ));

   // Declared test cases
   testCases.push_back( std::make_tuple(
     // Input: stiffness, damping, inertia, xd, x, xd_dot, x_dot, xd_dot_dot, Fd, F
     std::make_tuple("Imedance PD5",stiffness::Ones(2),
                     damping::Ones(2),
                     inertia::Ones(2),
                     0.0,
                     0.0,
                     0,
                     Xd::Ones(2),
                     X::Zero(2),
                     Xd_dot::Ones(2),
                     X_dot::Zero(2),
                     Xd_dot_dot::Zero(2),
                     Fd::Zero(2),
                     F::Ones(2)),
     // ExpectedResult: error, e, error_dot, U
     std::make_tuple(error::Ones(2), error_dot::Ones(2), U::Ones(2))
   ));

   // Declared test cases
   testCases.push_back( std::make_tuple(
     // Input: stiffness, damping, inertia, xd, x, xd_dot, x_dot, xd_dot_dot, Fd, F
     std::make_tuple("Imedance PD6",stiffness::Ones(2),
                     damping::Ones(2),
                     inertia::Ones(2),
                     0.0,
                     0.0,
                     0,
                     Xd::Ones(2),
                     X::Zero(2),
                     Xd_dot::Ones(2),
                     X_dot::Zero(2),
                     Xd_dot_dot::Ones(2) * 3,
                     Fd::Zero(2),
                     F::Ones(2)),
     // ExpectedResult: error, e, error_dot, U
     std::make_tuple(error::Ones(2), error_dot::Ones(2), U::Ones(2) * 4)
   ));
 };
};

/*****************************************************************************
 * Policy: NormStiffness
 *****************************************************************************/
struct NormStiffnessTestData : public GenericTestData
{
 //Make norm stiffness specific test cases
 NormStiffnessTestData()
 {
   // Declared test cases
   testCases.push_back( std::make_tuple(
     // Input: stiffness, damping, inertia, xd, x, xd_dot, x_dot, xd_dot_dot, Fd, F
     std::make_tuple("Imedance NormStiffness1",stiffness::Ones(2),
                     damping::Ones(2),
                     inertia::Ones(2),
                     1.0,
                     0.0,
                     0,
                     Xd::Zero(2),
                     X::Zero(2),
                     Xd_dot::Zero(2),
                     X_dot::Zero(2),
                     Xd_dot_dot::Zero(2),
                     Fd::Zero(2),
                     F::Zero(2)),
     // ExpectedResult: error, e, error_dot, U
     std::make_tuple(error::Zero(2), error_dot::Zero(2), U::Zero(2))
   ));

   // Declared test cases
   testCases.push_back( std::make_tuple(
     // Input: stiffness, damping, inertia, xd, x, xd_dot, x_dot, xd_dot_dot, Fd, F
     std::make_tuple("Imedance NormStiffness2",stiffness::Ones(2),
                     damping::Ones(2),
                     inertia::Ones(2),
                     1.0,
                     0.0,
                     0,
                     Xd::Zero(2),
                     X::Zero(2),
                     Xd_dot::Zero(2),
                     X_dot::Zero(2),
                     Xd_dot_dot::Zero(2),
                     Fd::Zero(2),
                     F::Ones(2)),
     // ExpectedResult: error, e, error_dot, U
     std::make_tuple(error::Zero(2), error_dot::Zero(2), U::Ones(2) * -1)
   ));

   // Declared test cases
   testCases.push_back( std::make_tuple(
     // Input: stiffness, damping, inertia, xd, x, xd_dot, x_dot, xd_dot_dot, Fd, F
     std::make_tuple("Imedance NormStiffness3",stiffness::Ones(2),
                     damping::Ones(2),
                     inertia::Ones(2) * 0.5,
                     1.0,
                     0.0,
                     0,
                     Xd::Zero(2),
                     X::Zero(2),
                     Xd_dot::Zero(2),
                     X_dot::Zero(2),
                     Xd_dot_dot::Zero(2),
                     Fd::Zero(2),
                     F::Ones(2)),
     // ExpectedResult: error, e, error_dot, U
     std::make_tuple(error::Zero(2), error_dot::Zero(2), U::Ones(2) * -2)
   ));

   // Declared test cases
   testCases.push_back( std::make_tuple(
     // Input: stiffness, damping, inertia, xd, x, xd_dot, x_dot, xd_dot_dot, Fd, F
     std::make_tuple("Imedance NormStiffness4",stiffness::Ones(2),
                     damping::Ones(2),
                     inertia::Ones(2),
                     10.0,
                     0.0,
                     0,
                     Xd::Ones(2),
                     X::Zero(2),
                     Xd_dot::Zero(2),
                     X_dot::Zero(2),
                     Xd_dot_dot::Zero(2),
                     Fd::Zero(2),
                     F::Ones(2)),
     // ExpectedResult: error, e, error_dot, U
     std::make_tuple(error::Ones(2), error_dot::Zero(2), U::Zero(2))
   ));

   // Declared test cases
   testCases.push_back( std::make_tuple(
     // Input: stiffness, damping, inertia, xd, x, xd_dot, x_dot, xd_dot_dot, Fd, F
     std::make_tuple("Imedance NormStiffness5",stiffness::Ones(2),
                     damping::Ones(2),
                     inertia::Ones(2),
                     1.0,
                     0.0,
                     0,
                     Xd::Ones(2),
                     X::Zero(2),
                     Xd_dot::Ones(2),
                     X_dot::Zero(2),
                     Xd_dot_dot::Zero(2),
                     Fd::Zero(2),
                     F::Ones(2)),
     // ExpectedResult: error, e, error_dot, U
     std::make_tuple(error::Ones(2), error_dot::Ones(2), U::Ones(2) * 1.0 / sqrt(2))
   ));

   // Declared test cases
   testCases.push_back( std::make_tuple(
     // Input: stiffness, damping, inertia, xd, x, xd_dot, x_dot, xd_dot_dot, Fd, F
     std::make_tuple("Imedance NormStiffness6",stiffness::Ones(2),
                     damping::Ones(2),
                     inertia::Ones(2),
                     1.0,
                     0.0,
                     0,
                     Xd::Ones(2),
                     X::Zero(2),
                     Xd_dot::Ones(2),
                     X_dot::Zero(2),
                     Xd_dot_dot::Ones(2) * 3,
                     Fd::Zero(2),
                     F::Ones(2)),
     // ExpectedResult: error, e, error_dot, U
     std::make_tuple(error::Ones(2), error_dot::Ones(2), U::Ones(2) * (3 + 1.0 / sqrt(2)))
   ));
 };
};

TEST(ImpedanceControllerTest, NormStiffness)
{
  NormStiffnessTestData testData;
  test_controller(ImpedancePolicy::NormStiffness, testData);
}

/*****************************************************************************
 * Policy: PiecewiseNormStiffness
 *****************************************************************************/
struct PiecewiseNormStiffnessTestData : public GenericTestData
{
 //Make norm stiffness specific test cases
 PiecewiseNormStiffnessTestData()
 {
   // Declared test cases
   testCases.push_back( std::make_tuple(
     // Input: stiffness, damping, inertia, xd, x, xd_dot, x_dot, xd_dot_dot, Fd, F
     std::make_tuple("Imedance PWNormStiffness1",stiffness::Ones(2),
                     damping::Ones(2),
                     inertia::Ones(2),
                     1.0,
                     1.0,
                     1,
                     Xd::Zero(2),
                     X::Zero(2),
                     Xd_dot::Zero(2),
                     X_dot::Zero(2),
                     Xd_dot_dot::Zero(2),
                     Fd::Zero(2),
                     F::Zero(2)),
     // ExpectedResult: error, e, error_dot, U
     std::make_tuple(error::Zero(2), error_dot::Zero(2), U::Zero(2))
   ));

   // Declared test cases
   testCases.push_back( std::make_tuple(
     // Input: stiffness, damping, inertia, xd, x, xd_dot, x_dot, xd_dot_dot, Fd, F
     std::make_tuple("Imedance PWNormStiffness2",stiffness::Ones(2),
                     damping::Ones(2),
                     inertia::Ones(2),
                     1.0,
                     1.0,
                     2,
                     Xd::Zero(2),
                     X::Zero(2),
                     Xd_dot::Zero(2),
                     X_dot::Zero(2),
                     Xd_dot_dot::Zero(2),
                     Fd::Zero(2),
                     F::Ones(2)),
     // ExpectedResult: error, e, error_dot, U
     std::make_tuple(error::Zero(2), error_dot::Zero(2), U::Ones(2) * -1)
   ));

   // Declared test cases
   testCases.push_back( std::make_tuple(
     // Input: stiffness, damping, inertia, xd, x, xd_dot, x_dot, xd_dot_dot, Fd, F
     std::make_tuple("Imedance PWNormStiffness3",stiffness::Ones(2),
                     damping::Ones(2),
                     inertia::Ones(2) * 0.5,
                     1.0,
                     1.0,
                     0,
                     Xd::Zero(2),
                     X::Zero(2),
                     Xd_dot::Zero(2),
                     X_dot::Zero(2),
                     Xd_dot_dot::Zero(2),
                     Fd::Zero(2),
                     F::Ones(2)),
     // ExpectedResult: error, e, error_dot, U
     std::make_tuple(error::Zero(2), error_dot::Zero(2), U::Ones(2) * -2)
   ));

   // Declared test cases
   testCases.push_back( std::make_tuple(
     // Input: stiffness, damping, inertia, xd, x, xd_dot, x_dot, xd_dot_dot, Fd, F
     std::make_tuple("Imedance PWNormStiffness4",stiffness::Ones(2),
                     damping::Ones(2),
                     inertia::Ones(2),
                     10.0,
                     10.0,
                     0,
                     Xd::Ones(2),
                     X::Zero(2),
                     Xd_dot::Zero(2),
                     X_dot::Zero(2),
                     Xd_dot_dot::Zero(2),
                     Fd::Zero(2),
                     F::Ones(2)),
     // ExpectedResult: error, e, error_dot, U
     std::make_tuple(error::Ones(2), error_dot::Zero(2), U::Zero(2))
   ));

   // Declared test cases
   testCases.push_back( std::make_tuple(
     // Input: stiffness, damping, inertia, xd, x, xd_dot, x_dot, xd_dot_dot, Fd, F
     std::make_tuple("Imedance PWNormStiffness5",stiffness::Ones(2),
                     damping::Ones(2),
                     inertia::Ones(2),
                     1.0,
                     1.0,
                     1,
                     Xd::Ones(2),
                     X::Zero(2),
                     Xd_dot::Ones(2),
                     X_dot::Zero(2),
                     Xd_dot_dot::Zero(2),
                     Fd::Zero(2),
                     F::Ones(2)),
     // ExpectedResult: error, e, error_dot, U
     std::make_tuple(error::Ones(2), error_dot::Ones(2), U::Ones(2) )
   ));

   // Declared test cases
   testCases.push_back( std::make_tuple(
     // Input: stiffness, damping, inertia, xd, x, xd_dot, x_dot, xd_dot_dot, Fd, F
     std::make_tuple("Imedance PWNormStiffness6",stiffness::Ones(2),
                     damping::Ones(2),
                     inertia::Ones(2),
                     1.0,
                     1.0,
                     2,
                     Xd::Ones(2),
                     X::Zero(2),
                     Xd_dot::Ones(2),
                     X_dot::Zero(2),
                     Xd_dot_dot::Ones(2) * 3,
                     Fd::Zero(2),
                     F::Ones(2)),
     // ExpectedResult: error, e, error_dot, U
     std::make_tuple(error::Ones(2), error_dot::Ones(2), U::Ones(2) * (3 + 1.0 / sqrt(2)))
   ));
 };
};

TEST(ImpedanceControllerTest, PiecewiseNormStiffness)
{
  PiecewiseNormStiffnessTestData testData;
  test_controller(ImpedancePolicy::PiecewiseNormStiffness, testData);
}

} // namespace task_library
} // namespace controlit