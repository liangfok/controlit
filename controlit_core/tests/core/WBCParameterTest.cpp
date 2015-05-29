#include <gtest/gtest.h>

#include <controlit/addons/cppStatus.hpp>
#include <controlit/utility/ControlItParameters.hpp>
#include <controlit_robot_models/WBCJointLimits.hpp>

using controlit::addons::cpp::Status;
using Vector;
using controlit::rbdl_robot_urdfreader::WBCJointLimits;

class WBCParameterTest : public ::testing::Test
{
protected:
    virtual void SetUp() {}
    virtual void TearDown() {}
};

/* ----------------------------------------------------------------------------
 *
 * --------------------------------------------------------------------------*/

TEST_F(WBCParameterTest, BasicTest)
{
    // Create a ControlItParameters object
    controlit::utility::ControlItParameters controlitParameters;
  
    // Verify that the default gravity parameter is correct
    Vector gravity(3);
    gravity << 0, 0, -9.81;
  
    EXPECT_TRUE(gravity.isApprox(controlitParameters.getGravityVector(), 0.01))
        << "Gravity vector mismatch! Got " << controlitParameters.getGravityVector().transpose()
        << ", expected " << gravity.transpose();
  
    // Arbitrary select numDOFs = 5
    int numDOFs = 5;
  
    // Create a ROS node handle and initialize controlitParameters
    ros::NodeHandle nh;
    EXPECT_TRUE(controlitParameters.init(nh));
  
    // Create a WBCJointLimits object.  Set the joint limits in it, then update the ControlItParameters object
    // with the joint limits.
    WBCJointLimits limits;
    limits.init(numDOFs);
    limits.torqueLimits << 1.1, 2.2, 3.3, 4.4, 5.5;
    limits.velocityLimits << 9, 8, 7, 6, 5;
    limits.positionLowerLimits << 2, 4, 5, 6, 8;
    limits.positionUpperLimits << 12, 14, 15, 16, 18;
  
    controlitParameters.saveLimits(limits);
  
    EXPECT_TRUE(controlitParameters.checkParameters(numDOFs));
  
    // Verify that the torque limits are set and are correct
    EXPECT_TRUE(controlitParameters.hasTorqueLimits()) << "WBC Parameters did not have torque limits!";
  
    Vector torqueLimits(numDOFs);
    torqueLimits << 1.1, 2.2, 3.3, 4.4, 5.5;
  
    EXPECT_TRUE(torqueLimits.isApprox(controlitParameters.getTorqueLimits(), 1e-10))
        << "Torque limit vector mismatch! Got " << controlitParameters.getTorqueLimits().transpose()
        << ", expected " << torqueLimits.transpose();
  
    // Verify that the torque offsets are set and are correct
    EXPECT_TRUE(controlitParameters.hasTorqueOffsets()) << "WBC Parameters did not have torque offsets!";
  
    Vector torqueOffsets(numDOFs);
    torqueOffsets << 2.1, -9.2, 10.6, -55.2, 23.1;
  
    EXPECT_TRUE(torqueOffsets.isApprox(controlitParameters.getTorqueOffsets(), 1e-10))
        << "Torque offset vector mismatch! Got " << controlitParameters.getTorqueOffsets().transpose()
        << ", expected " << torqueOffsets.transpose();
  
    // Verify that the torque scaling factors are set and are correct
    EXPECT_TRUE(controlitParameters.hasTorqueScalingFactors()) << "WBC Parameters did not have torque scaling factors!";
  
    Vector torqueScalingFactors(numDOFs);
    torqueScalingFactors << -1.1, 1.1, -2.2, 2.2, -3.3;
  
    EXPECT_TRUE(torqueScalingFactors.isApprox(controlitParameters.getTorqueScalingFactors(), 1e-10))
        << "Torque scaling factor mismatch! Got " << controlitParameters.getTorqueScalingFactors().transpose()
        << ", expected " << torqueScalingFactors.transpose();
  
    // Verify that the gravity vector is correct
    gravity << 0, 0, 10.10;
  
    EXPECT_TRUE(gravity.isApprox(controlitParameters.getGravityVector(), 1e-10))
        << "Gravity mismatch! Got " << controlitParameters.getGravityVector().transpose()
        << ", expected " << gravity.transpose();
  
    // Verify that the ramp up time is set and is correct
    EXPECT_TRUE(controlitParameters.hasRampUpTime()) << "WBC Parameters did not have ramp up time specified!";
  
    EXPECT_TRUE(controlitParameters.getRampUpTime() == 4.6)
        << "Ramp up time mismatch! Got " << controlitParameters.getRampUpTime()
        << ", expected 4.6";
  
    // Verify that the coupledJointGroups is set and is correct
    EXPECT_TRUE(controlitParameters.hasCoupledJointGroups()) << "WBC Parameters did not have coupled joint groups specified!";
  
    if (controlitParameters.hasCoupledJointGroups())
    {
        std::vector<std::string> vec1;
        //vec1.resize(2);
        vec1.push_back("foo");
        vec1.push_back("bar");
    
        std::vector<std::string> vec2;
        //vec2.resize(1);
        vec2.push_back("baz");
    
        std::vector<std::string> vec3;
        //vec3.resize(2);
        vec3.push_back("qux");
        vec3.push_back("quux");
    
        std::vector<std::vector<std::string>> expectedCoupledJointGroups;
        //expectedCoupledJointGroups.resize(3);
        expectedCoupledJointGroups.push_back(vec1);
        expectedCoupledJointGroups.push_back(vec2);
        expectedCoupledJointGroups.push_back(vec3);
    
        std::vector<std::vector<std::string>> coupledJointGroups;
        coupledJointGroups = controlitParameters.getCoupledJointGroups();
    
        for (size_t ii = 0; ii < expectedCoupledJointGroups.size(); ii++)
        {
            std::cout<<"Parsed group "<<ii<<": "<<std::endl;
            for (size_t jj = 0; jj < expectedCoupledJointGroups[ii].size(); jj++)
            {
                std::cout<<coupledJointGroups[ii][jj]<<", ";
                EXPECT_TRUE(expectedCoupledJointGroups[ii][jj].compare(coupledJointGroups[ii][jj]) == 0)
                << "CoupleJointGroups mismatch at index [" << ii << "][" << jj << "]! Got " << coupledJointGroups[ii][jj]
                << ", expected " << expectedCoupledJointGroups[ii][jj];
            }
            std::cout<<std::endl;
        }
    }
  
    // Verify that the gravity compensation mask is set and is correct
    EXPECT_TRUE(controlitParameters.hasGravCompMask()) << "WBC Parameters did not have gravity compensation mask specified!";
  
    if (controlitParameters.hasGravCompMask())
    {
        std::vector<std::string> expectedGravCompMask;
        expectedGravCompMask.push_back("foo");
        expectedGravCompMask.push_back("bar");
        expectedGravCompMask.push_back("baz");
        expectedGravCompMask.push_back("qux");
    
        std::vector<std::string> gravCompMask;
        gravCompMask = controlitParameters.getGravCompMask();
    
        for (size_t ii = 0; ii < expectedGravCompMask.size(); ii++)
        {
            EXPECT_TRUE(expectedGravCompMask[ii].compare(gravCompMask[ii]) == 0)
                << "Gravity compensation mask mismatch at index [" << ii << "]! Got " << gravCompMask[ii]
                << ", expected " << expectedGravCompMask[ii];
        }
    }
  
    // Verify that the effective gain pass through mask is set and is correct
    EXPECT_TRUE(controlitParameters.hasEffectiveGainPassThroughMask())
        << "WBC Parameters did not have effective gain pass through mask specified!";
  
    if (controlitParameters.hasEffectiveGainPassThroughMask())
    {
        std::vector<std::string> expectedGainMask;
        expectedGainMask.push_back("amy");
        expectedGainMask.push_back("jon");
        expectedGainMask.push_back("bob");
        expectedGainMask.push_back("lou");
        expectedGainMask.push_back("kat");
    
        std::vector<std::string> gainMask;
        gainMask = controlitParameters.getEffectiveGainPassThroughMask();
    
        for (size_t ii = 0; ii < expectedGainMask.size(); ii++)
        {
            EXPECT_TRUE(expectedGainMask[ii].compare(gainMask[ii]) == 0)
              << "Expected gain pass through mask mismatch at index [" << ii << "]! Got " << gainMask[ii]
              << ", expected " << expectedGainMask[ii];
        }
    }
  
    // Verify the update rate was set
    EXPECT_TRUE(controlitParameters.getUpdateRate() == 211)
        << "Update rate mismatch! Got " << controlitParameters.getUpdateRate()
        << ", expected 211";
  
    // Verify the max effort command was set
    EXPECT_TRUE(controlitParameters.getMaxEffortCmd() == 123.456)
        << "Incorrect max effort command! Got " << controlitParameters.getMaxEffortCmd()
        << ", expected 123.456";
  
    // Verify the limit enforcement variables are correctly set
    EXPECT_TRUE(controlitParameters.enforceEffortLimits()) << "Parameter 'enforce_effort_limits' not correctly set.";
    EXPECT_TRUE(controlitParameters.enforceVelocityLimits()) << "Parameter 'enforce_velocity_limits' not correctly set.";
    EXPECT_TRUE(controlitParameters.enforceLowerPositionLimits()) << "Parameter 'enforce_lower_position_limits' not correctly set.";
    EXPECT_TRUE(controlitParameters.enforceUpperPositionLimits()) << "Parameter 'enforce_upper_position_limits' not correctly set.";
}