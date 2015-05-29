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

#ifndef __CONTROLIT_JOINT_LIMITS__
#define __CONTROLIT_JOINT_LIMITS__

#include <Eigen/Dense>

namespace controlit {
namespace rbdl_robot_urdfreader {


class WBCJointLimits
{
public:
  WBCJointLimits(){}
  void init(int numDOFs); //number of actuable dofs

  /*!
  * The joint torque limits.
  */
  Eigen::VectorXd torqueLimits;

  /*!
  * The joint velocity limits.
  */
  Eigen::VectorXd velocityLimits;

  /*!
  * The joint lower position limits.
  */
  Eigen::VectorXd positionLowerLimits;

  /*!
  * The joint upper position limits.
  */
  Eigen::VectorXd positionUpperLimits;
};

} // namespace rbdl_robot_urdfreader
} // namespace controlit


#endif  // __CONTROLIT_JOINT_LIMITS__
