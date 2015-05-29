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

#ifndef __CONTROLIT_CONTAINER_UTILITY_HPP__
#define __CONTROLIT_CONTAINER_UTILITY_HPP__

#include <string>
#include <rbdl/rbdl.h>

#include <controlit/PlanElement.hpp>

namespace controlit {
namespace utility {

/*!
 * Utility functions for analyzing and working with containers.
 * Currently the containers supported are Vector and Matrix.
 */
class ContainerUtility
{
public:
    /*!
     * The constructor.
     */
    ContainerUtility();
  
    /*!
     * Checks the elements in the supplied vector to ensure that
     * they fall within the specified minVal and maxVal.
     *
     * \param vv The vector to check
     * \param minVal The minimum value.
     * \param maxVal The maximum value.
     * \return true if all elements in vv fall between the minVal and
     * maxVal.
     */
    static bool checkRange(Vector & vv, double minVal, double maxVal);
  
    /*!
     * Checks the magnitude of the elements in the supplied vector.
     * Ensures that they fall below the specified magnitude.
     *
     * \param vv The vector to check.
     * \param magnitude The specified magnitude.
     * \return true if the magnitude of all elements in vv are
     * below the magnitude.
     */
    static bool checkMagnitude(Vector & vv, double magnitude);
  
    /*!
     * Checks whether the supplied matrix contains any values
     * above or below a certain value.
     *
     * \param mm The matrix to check.
     * \param minVal The minimum acceptable value.
     * \param maxVal The maximum acceptable value.
     * \return true if all values in the vector are between
     * or equal to the minVal and maxVal.
     */
    static bool checkRange(Matrix & mm, double minVal, double maxVal);
  
    /*!
     * Checks whether the supplied matrix contains any values
     * whose absolute value exceeds a certain magnitude.
     *
     * \param mm The vector to check.
     * \param magnitude The maximum magnitude.
     * \return true if the absolute values of all elements in the vector are
     * less than or equal to the specified magnitude.
     */
    static bool checkMagnitude(Matrix & mm, double magnitude);
  
    /*!
     * Checks whether the supplied matrix contains any values
     * above or below a certain value.
     *
     * \param mm The matrix to check.
     * \param minVal The minimum acceptable value.
     * \param maxVal The maximum acceptable value.
     * \return true if all values in the vector are between
     * or equal to the minVal and maxVal.
     */
    static bool checkRange(Eigen::Matrix3d & mm, double minVal, double maxVal);
  
    /*!
     * Checks whether the supplied matrix contains any values
     * whose absolute value exceeds a certain magnitude.
     *
     * \param mm The vector to check.
     * \param magnitude The maximum magnitude.
     * \return true if the absolute values of all elements in the vector are
     * less than or equal to the specified magnitude.
     */
    static bool checkMagnitude(Eigen::Matrix3d & mm, double magnitude);
};

} // namespace utility
} // namespace controlit

#endif // __CONTROLIT_CONTAINER_UTILITY_HPP__
