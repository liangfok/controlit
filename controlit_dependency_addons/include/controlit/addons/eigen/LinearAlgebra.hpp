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

#ifndef __CONTROLIT_ADDONS_EIGEN_LINEAR_ALGEBRA__
#define __CONTROLIT_ADDONS_EIGEN_LINEAR_ALGEBRA__

#include <ros/console.h>

// Test to ensure no dynamic memory allocation is being done
// #define EIGEN_RUNTIME_NO_MALLOC 1

// Must include RBDL first!
#include <rbdl/rbdl.h>
#include <rbdl/Dynamics.h>
#include <rbdl/Kinematics.h>
#include <rbdl/rbdl_mathutils.h>

#include <Eigen/Dense>

// #include <controlit/logging/Logging.hpp>

namespace controlit {
namespace addons {
namespace eigen {

typedef Eigen::VectorXd Vector;
typedef Eigen::MatrixXd Matrix;

typedef Eigen::Vector3d Vector3d;
typedef Eigen::Matrix3d Matrix3d;

typedef Eigen::Quaterniond Quaternion;

// Should this be moved into controlit_addons_rbdl?
const unsigned int RBDL_BODY_NOT_FOUND = std::numeric_limits<unsigned int>::max();

/**
 * Checks the values in a matrix to determine whether it contains invalid values.
 *
 * \param mm The matrix to check.
 * \param minVal The minimum acceptable value.
 * \param maxVal The maximum acceptable value.
 * \return true if all values in the matrix are between or equal to the minVal and maxVal.
*/
template<typename DerivedA>
bool checkRange(const Eigen::MatrixBase<DerivedA>& mm,
  typename DerivedA::Scalar minVal, typename DerivedA::Scalar maxVal)
{
    assert(minVal < maxVal);

    // Go through each element in the matrix and ensure they are not
    // NaN and fall within the specified min and max values.
    for (int ii = 0; ii < mm.rows(); ii++)
    {
        for (int jj = 0; jj < mm.cols(); jj++)
        {
            if (std::isnan(mm(ii, jj)))
            {
                std::stringstream ss;
                ss << "NaN detected at index (" << ii << ", " << jj << "), returning false!";
                ROS_WARN_STREAM(ss.str());
                return false;
            }
            if (mm(ii, jj) > maxVal || mm(ii, jj) < minVal)
            {
                std::stringstream ss;
                ss << "Value of out range at index (" << ii << ", " << jj << "), returning false.\n"
                     " - value = " << mm(ii, jj) << "\n"
                     " - minVal = " << minVal << "\n"
                     " - maxVal = " << maxVal;
                ROS_WARN_STREAM(ss.str());
                return false;
            }
        }
    }

    return true;
}

/*!
 * Checks whether the supplied matrix contains any values
 * above or below a certain value.
 *
 * \param mm The matrix to check.
 * \param magnitude The maximum magnitude of a value in the matrix.
 * \return true if the absolute value of all values in the matrix is less than the magnitude.
 */
template<typename DerivedA>
bool checkMagnitude(const Eigen::MatrixBase<DerivedA>& mm,
  typename DerivedA::Scalar magnitude = 1e10)
{
    return checkRange(mm, -1 * std::abs(magnitude), std::abs(magnitude));
}

/**
 * Checks the values in a quaternion to determine whether it contains invalid values.
 *
 * \param qq The quaternion to check.
 * \param minVal The minimum acceptable value.
 * \param maxVal The maximum acceptable value.
 * \return true if all values in the quaternion are between or equal to the minVal and maxVal.
*/
template<typename DerivedA>
bool checkRange(const Eigen::Quaternion<DerivedA>& qq,
  double minVal, double maxVal)
{
    assert(minVal < maxVal);

    if (std::isnan(qq.w()) || std::isnan(qq.x()) || std::isnan(qq.y()) || std::isnan(qq.z()))
    {
        std::stringstream ss;
        ss << "Quaternion [" << qq.w() << ", " << qq.x() << ", " << qq.y() << ", " << qq.z()  << "] contains NaN values!\n";
        ROS_WARN_STREAM(ss.str());
        return false;
    }

    if (qq.w() > maxVal || qq.w() < minVal ||
        qq.x() > maxVal || qq.x() < minVal ||
        qq.y() > maxVal || qq.y() < minVal ||
        qq.z() > maxVal || qq.z() < minVal)
    {
        std::stringstream ss;
        ss << "Quaternion contains terms out of range:\n"
          << " - w: [" << qq.w() << ", " << qq.x() << ", " << qq.y() << ", " << qq.z()  << "]\n"
          << " - minVal: " << minVal << "\n"
          << " - maxVel: " << maxVal;
        ROS_WARN_STREAM(ss.str());
    }

    return true;
}

/*!
 * Checks whether the supplied quaternion contains any values
 * above or below a certain value.
 *
 * \param qq The quaternion to check.
 * \param magnitude The maximum magnitude of a value in the quaternion.
 * \return true if the absolute value of all values in the quaternion is less than the magnitude.
 */
template<typename DerivedA>
bool checkMagnitude(const Eigen::Quaternion<DerivedA>& qq, double magnitude = 1e10)
{
    return checkRange(qq, -1 * std::abs(magnitude), std::abs(magnitude));
}

} // namespace eigen
} // namespace addons
} // namespace controlit

#endif // __CONTROLIT_ADDONS_EIGEN_LINEAR_ALGEBRA__
