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

#include <controlit/utility/ContainerUtility.hpp>
#include <controlit/logging/RealTimeLogging.hpp>
#include <controlit/addons/eigen/LinearAlgebra.hpp>

namespace controlit {
namespace utility {

using controlit::addons::eigen::Vector;
using controlit::addons::eigen::Matrix;
using controlit::addons::eigen::Matrix3d;

ContainerUtility::ContainerUtility()
{
}

bool ContainerUtility::checkRange(Vector & vv, double minVal, double maxVal)
{
  assert(minVal < maxVal);

  for (int ii = 0; ii < vv.size(); ii++)
  {
    if (isnan(vv(ii)))
    {
      CONTROLIT_WARN << "(Vector): NaN detected at index " << ii << ", returning false!";
      return false;
    }
    if (vv(ii) > maxVal || vv(ii) < minVal)
    {
      CONTROLIT_WARN << "(Vector): Value out of range at index " << ii << ", returning false.\n"
           " - value = " << vv(ii) << "\n"
           " - minVal = " << minVal << "\n"
           " - maxval = " << maxVal;
      return false;
    }
  }

  return true;
}

bool ContainerUtility::checkMagnitude(Vector & vv, double magnitude)
{
  return checkRange(vv, -1 * std::abs(magnitude), std::abs(magnitude));
}

bool ContainerUtility::checkRange(Matrix & mm, double minVal, double maxVal)
{
  assert(minVal < maxVal);

  // Go through each element in the matrix and ensure they are not
  // NaN and fall within the specified min and max values.
  for (int ii = 0; ii < mm.rows(); ii++)
  {
    for (int jj = 0; jj < mm.cols(); jj++)
    {
      if (isnan(mm(ii, jj)))
      {
        CONTROLIT_WARN << "(Matrix): NaN detected at index (" << ii << ", " << jj << "), returning false!";
        return false;
      }
      if (mm(ii, jj) > maxVal || mm(ii, jj) < minVal)
      {
        CONTROLIT_WARN << "(Matrix): Value of out range at index (" << ii << ", " << jj << "), returning false.\n"
             " - value = " << mm(ii, jj) << "\n"
             " - minVal = " << minVal << "\n"
             " - maxVal = " << maxVal;
        return false;
      }
    }
  }

  return true;
}

bool ContainerUtility::checkMagnitude(Matrix & mm, double magnitude)
{
  return checkRange(mm, -1 * std::abs(magnitude), std::abs(magnitude));
}

bool ContainerUtility::checkRange(Matrix3d & mm, double minVal, double maxVal)
{
  assert(minVal < maxVal);

  // Go through each element in the matrix and ensure they are not
  // NaN and fall within the specified min and max values.
  for (int ii = 0; ii < mm.rows(); ii++)
  {
    for (int jj = 0; jj < mm.cols(); jj++)
    {
      if (isnan(mm(ii, jj)))
      {
        CONTROLIT_WARN << "(Matrix): NaN detected at index (" << ii << ", " << jj << "), returning false!";
        return false;
      }
      if (mm(ii, jj) > maxVal || mm(ii, jj) < minVal)
      {
        CONTROLIT_WARN << "(Matrix): Value of out range at index (" << ii << ", " << jj << "), returning false.\n"
             " - value = " << mm(ii, jj) << "\n"
             " - minVal = " << minVal << "\n"
             " - maxVal = " << maxVal;
        return false;
      }
    }
  }

  return true;
}

bool ContainerUtility::checkMagnitude(Matrix3d & mm, double magnitude)
{
  return checkRange(mm, -1 * std::abs(magnitude), std::abs(magnitude));
}

} // namespace utility
} // namespace controlit
