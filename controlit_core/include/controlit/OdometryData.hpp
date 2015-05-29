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

#ifndef __CONTROLIT_CORE_ODOMETRY_DATA_HPP__
#define __CONTROLIT_CORE_ODOMETRY_DATA_HPP__

#include <controlit/addons/eigen/LinearAlgebra.hpp>

namespace controlit {

using controlit::addons::eigen::Vector;
using controlit::addons::eigen::Matrix;
using controlit::addons::eigen::Vector3d;
using controlit::addons::eigen::Matrix3d;

/*!
 * Odometry data.  This is used to synchronize the updating
 * of the virtual joints of the robot and the robot model.
 */
struct OdometryData
{
    OdometryData() :
        x(Vector3d::Zero()),
        q(Eigen::Quaterniond::Identity()),
        x_dot(Vector::Zero(6)) {}

    OdometryData(Vector3d const& x_in, Eigen::Quaterniond const& q_in, Vector const& x_dot_in) :
        x(x_in),
        q(q_in),
        x_dot(x_dot_in) {}

    OdometryData(OdometryData const& odomData) :
        x(odomData.x),
        q(odomData.q),
        x_dot(odomData.x_dot) {}

    OdometryData& operator=(OdometryData const& rhs)
    {
        if (this == &rhs) return *this;
        x = rhs.x;
        q = rhs.q;

        // Assigment only.. should be RT.
        assert(x_dot.rows() == rhs.x_dot.rows());
        x_dot = rhs.x_dot;

        return *this;
    }

    Vector3d x;
    Eigen::Quaterniond q;
    Vector x_dot;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace controlit

#endif // __CONTROLIT_CORE_ODOMETRY_DATA_HPP__