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

#ifndef __CONTROLIT_ADDONS_ROS_MSG_EIGEN_CONVERSION_HPP__
#define __CONTROLIT_ADDONS_ROS_MSG_EIGEN_CONVERSION_HPP__

#include <string>
#include <std_msgs/Float64MultiArray.h>


// Below are methods for converting between ROS messages and Eigen data types.

namespace controlit {
namespace addons {
namespace ros {

template <class Derived>
void matrixEigenToMsg(const Eigen::MatrixBase<Derived> &e, std_msgs::Float64MultiArray &m)
{
  if (m.layout.dim.size() != 2)
    m.layout.dim.resize(2);
  m.layout.dim[0].stride = e.rows() * e.cols();
  m.layout.dim[0].size = e.rows();
  m.layout.dim[1].stride = e.cols();
  m.layout.dim[1].size = e.cols();
  if ((int)m.data.size() != e.size())
    m.data.resize(e.size());
  int ii = 0;
  for (int i = 0; i < e.rows(); ++i)
    for (int j = 0; j < e.cols(); ++j)
      m.data[ii++] = e.coeff(i, j);
}

template <class Derived>
void matrixMsgToEigen(const std_msgs::Float64MultiArray &m, Eigen::MatrixBase<Derived> &e)
{
  int rows = m.layout.dim[0].size; // rows
  int cols = 1;
  if (m.layout.dim.size() > 1)
    cols = m.layout.dim[1].size; // cols

  e.resize(rows,cols);

  int ij = 0;
  for (int i = 0; i < rows; ++i)
    for (int j = 0; j < cols; ++j)
      e(i, j) = m.data[ij++];
}

} // namespace ros
} // namespace addons
} // namespace controlit

#endif // __CONTROLIT_ADDONS_ROS_MSG_EIGEN_CONVERSION_HPP__
