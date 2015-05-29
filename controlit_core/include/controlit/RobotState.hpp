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

#ifndef __CONTROLIT_ROBOT_STATE__
#define __CONTROLIT_ROBOT_STATE__


#include <ros/ros.h>

#include <chrono>
#include <controlit/addons/cpp/Assert.hpp>
#include <controlit/addons/eigen/LinearAlgebra.hpp>

using controlit::addons::eigen::Vector;
using controlit::addons::eigen::Matrix3d;

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;

#define NUM_VIRTUAL_DOFS 6

namespace controlit {

class RobotState
{
public:
  /*!
   * The constructor.
   */
  RobotState();

  /**
   * Initializes this class.  Resizes member variables to match the number of DOFs
   * in the robot.
   *
   * \param[in] jointNames The names of the joints that are being controlled.
   */
  void init(const std::vector<std::string> & jointNames);

  /*!
   * Resets the timestamp to be the time when this method is called.
   */
  void resetTimestamp();

  /*!
   * Returns the timestamp of this RobotState object.
   *
   * \return The timestamp of this RobotState object.
   */
  const high_resolution_clock::time_point & getTimestamp() const;

  /*!
   * Returns the number of real joints in the robot.
   *
   * \return The number of real joints in the robot.
   */
  size_t getNumJoints() const;

  /*!
   * Sets the position of a particular joint.
   *
   * \param index The index of the joint to set.
   * \param position The position of the joint.
   * \return Whether the set operation was successful.
   */
  bool setJointPosition(int index, double position);

  /*!
   * Sets the velocity of a particular joint.
   *
   * \param index The index of the joint to set.
   * \param velocity The velocity of the joint.
   * \return Whether the set operation was successful.
   */
  bool setJointVelocity(int index, double velocity);

  /*!
   * Sets the acceleration of a particular joint.
   *
   * \param index The index of the joint to set.
   * \param acceleration The acceleration of the joint.
   * \return Whether the set operation was successful.
   */
  bool setJointAcceleration(int index, double acceleration);

  /*!
   * Sets the effort of a particular joint.
   *
   * \param index The index of the joint to set.
   * \param effort The effort of the joint.
   * \return Whether the set operation was successful.
   */
  bool setJointEffort(int index, double effort);

  /*!
   * Updates state of the robot's base body in the /world frame.
   * This is done at the same frequency as the main servo loop.
   *
   * \return Whether the base state was successfully set.
   */
  bool setRobotBaseState(Eigen::Vector3d const& x, Eigen::Quaterniond const& q, Vector const& x_dot);

  /*!
   * Returns the index of the joint with the specified name.
   *
   * \param name The name of the joint.
   * \return The index of the joint, or -1 if the joint could not be found.
   */
  int getJointIndex(const std::string name) const;

  /*!
   * An accessor to the list of joints that are being controlled.
   * The order matches that of the model.
   *
   * \return A list of joint names.
   */
  const std::vector<std::string> & getJointNames() const;

  /*!
   * An accessor to the joint positions.
   *
   * \return A reference to the vector containing the joint positions.
   */
  const Vector & getJointPosition() const;

  /*!
   * An accessor to the joint velocities.
   *
   * \return A reference to the vector containing the joint velocities.
   */
  const Vector & getJointVelocity() const;

  /*!
   * An accessor to the joint accelerations.
   *
   * \return A reference to the vector containing the joint accelerations.
   */
  const Vector & getJointAcceleration() const;

  /*!
   * An accessor to the joint effort.
   *
   * \return A reference to the vector containing the joint effort.
   */
  const Vector & getJointEffort() const;

  /*!
   * An accessor to the virtual joint positions.
   *
   * \return A reference to the vector containing the virtual joint positions.
   */
  const Eigen::Matrix< double, NUM_VIRTUAL_DOFS, 1 > & getVirtualJointPosition() const;

  /*!
   * An accessor to the virtual joint velocities.
   *
   * \return A reference to the vector containing the virtual joint velocities.
   */
  const Eigen::Matrix< double, NUM_VIRTUAL_DOFS, 1 > & getVirtualJointVelocity() const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

  std::vector<std::string> jointNames;

  /*!
   * The time when this RobotState was last updated.
   * This is useful to tell how old the RobotState is.
   */
  high_resolution_clock::time_point timestamp;

  /*!
   * The robot's joint positions.
   */
  Vector jointPosition;

  /*!
   * The robot's joint velocities.
   */
  Vector jointVelocity;

  /*!
   * The robot's joint accelerations.
   */
  Vector jointAcceleration;

  /*!
   * The robot's joint effort.
   */
  Vector jointEffort;

  /*!
   * The positions of the virtual joints in the robot.
   * The matrix has 6 rows and 1 column.
   */
  Eigen::Matrix< double, NUM_VIRTUAL_DOFS, 1 > virtualJointPosition;

  /*!
   * The velocities of the virtual joints in the robot.
   * The matrix has 6 rows and 1 column.
   */
  Eigen::Matrix< double, NUM_VIRTUAL_DOFS, 1 > virtualJointVelocity;
};

} // namespace controlit

#endif // __CONTROLIT_ROBOT_STATE__
