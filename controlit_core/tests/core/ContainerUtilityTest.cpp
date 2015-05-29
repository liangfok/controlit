#include <gtest/gtest.h>
#include <controlit/utility/ContainerUtility.hpp>
#include <cmath>

TEST(ContainerUtilityTest, MainTest)
{
  // Vector contains valid values
  Vector v1(5);
  v1 << 2, -1, 0, 1, 2;

  EXPECT_TRUE(controlit::utility::ContainerUtility::checkMagnitude(v1, 5))
    << "Vector v1 contains elements with magnitude greather than 5!\n"
    << " - v1 = " << v1.transpose();

  EXPECT_TRUE(controlit::utility::ContainerUtility::checkMagnitude(v1, 2))
    << "Vector v1 contains elements with magnitude greather than 2!\n"
    << " - v1 = " << v1.transpose();

  EXPECT_FALSE(controlit::utility::ContainerUtility::checkMagnitude(v1, 1))
    << "checkMagnitude failed to find element with magnitude > 1!\n"
    << " - v1 = " << v1.transpose();

  // Vector contains NaN
  Vector v2(3);
  v2 << sqrt(-1), 0, 0;

  EXPECT_FALSE(controlit::utility::ContainerUtility::checkMagnitude(v2, 5))
    << "checkMagnitude failed to detect NaN condition";
}
