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

#include <controlit/RobotState.hpp>
#include <controlit/logging/RealTimeLogging.hpp>
#include <controlit/addons/eigen/LinearAlgebra.hpp>
// #include <drc/eigen/addons.hpp>

namespace controlit {

using controlit::addons::eigen::Vector;
using controlit::addons::eigen::Vector3d;
using controlit::addons::eigen::Matrix3d;
using controlit::addons::eigen::Quaternion;


#define LOG_ERROR_STATEMENT(ss) CONTROLIT_ERROR << ss;

RobotState::RobotState()
{
}

void RobotState::init(const std::vector<std::string> & jointNames)
{
    size_t numDOFs = jointNames.size();

    assert(numDOFs > 0);

    jointPosition.setZero(numDOFs);
    jointVelocity.setZero(numDOFs);
    jointAcceleration.setZero(numDOFs);
    jointEffort.setZero(numDOFs);

    // Initialize the virtual joint position and velocity values to be zero.
    virtualJointPosition.setZero(virtualJointPosition.size());
    virtualJointVelocity.setZero(virtualJointVelocity.size());

    // Save the names of the joints
    this->jointNames.resize(numDOFs);
    for (size_t ii = 0; ii < numDOFs; ii++)
    {
        this->jointNames[ii] = jointNames[ii];
    }
}

int RobotState::getJointIndex(const std::string name) const
{
    for (int ii = 0; ii < (int)jointNames.size(); ii++)
    {
        if (jointNames[ii].compare(name) == 0)
            return ii;
    }

    CONTROLIT_ERROR << "Unable to find joint " << name;
    return -1;
}

void RobotState::resetTimestamp()
{
    timestamp = high_resolution_clock::now();
}

const high_resolution_clock::time_point & RobotState::getTimestamp() const
{
    return timestamp;
}

size_t RobotState::getNumJoints() const
{
    controlit_assert_msg(jointPosition.size() >= 0, "Failed to convert signed int " << jointPosition.size() << " to size_t.");
    return  (size_t)jointPosition.size();
}

bool RobotState::setJointPosition(int index, double position)
{
    if (index < jointPosition.size())
    {
        jointPosition(index) = position;
        return true;
    }
    else
    {
        CONTROLIT_ERROR << "Index " << index << " out of bounds, number of joints = " << jointPosition.size();
        return false;
    }
}

bool RobotState::setJointVelocity(int index, double velocity)
{
    if (index < jointVelocity.size())
    {
        jointVelocity(index) = velocity;
        return true;
    }
    else
    {
        CONTROLIT_ERROR << "Index " << index << " out of bounds, number of joints = " << jointVelocity.size();
        return false;
    }
}

bool RobotState::setJointAcceleration(int index, double acceleration)
{
    if (index < jointAcceleration.size())
    {
        jointAcceleration(index) = acceleration;
        return true;
    }
    else
    {
        CONTROLIT_ERROR << "Index " << index << " out of bounds, number of joints = " << jointAcceleration.size();
        return false;
    }
}

bool RobotState::setJointEffort(int index, double effort)
{
    if (index < jointEffort.size())
    {
        jointEffort(index) = effort;
        return true;
    }
    else
    {
        LOG_ERROR_STATEMENT("Index " << index << " out of bounds, number of joints = " << jointEffort.size())
        return false;
    }
}

bool RobotState::setRobotBaseState(Vector3d const& x, Quaternion const& q,
    Vector const& x_dot)
{
    // R_world_vtip represents the measured orientation of the virtual frame's tip
    // experssed in world coordinates
    // TODO: Cache R_world_vtip instead of allocating.
    Matrix3d R_world_vtip = q.toRotationMatrix(); // Read rotation matrix that expresses 'vtip' frame in 'world' coordinates

    // Virual link is a x-y-z translation, then a 3-2-1 orientation as specified in
    // its construction. See construct_model() in rbdl_robot_urdfreader.cpp. Specifically
    // how 'floating' joints are handled.
    Vector eulerAngles = R_world_vtip.eulerAngles(2,1,0);

    // Have everything we need to set the position/orientation from measured data
    virtualJointPosition.topRows(3) = x;
    virtualJointPosition.segment(3,3) = eulerAngles;

    // Now to compute the virtual link's velocities from the twist (x_dot) that was measured.
    // Again, it is assumed that x_dot is the twist of the vtip frame relative to the world frame.

    // Linear velocities are easy enough
    virtualJointVelocity.topRows(3) = x_dot.topRows(3);

    // Computing joint velocities takes advantage of the fact that angular
    // velocities are additive (omega_x_y is the angular velocity of frame 'y'
    // measured in reference to frame 'x')
    //
    //    omega_world_vtip = omega_world_joint1 + R_world_joint1 * omega_joint1_joint2 + R_world_joint2 * omega_joint2_vtip   (1)
    //
    // Expanding (1) and using that omega_world_joint1 is about local z-axis,
    // omega_joint1_joint2 is about the local y-axis, and so forth, we get an
    //
    //    A * x = b
    //
    // Where A is,
    //
    //    A = [z_world_joint1 y_world_joint1 x_world_joint2]
    //
    // and x is Qd_(3, 4, 5) and b is the angular part of x_dot
    // TODO: this could all be speed up if someone has the patience to solve the linear equality
    // by hand. It would be much faster..

    // TODO: Cache R_world_joint1 instead of allocating.
    Matrix3d R_world_joint1( Eigen::AngleAxisd(virtualJointPosition(3), Eigen::Vector3d::UnitZ()) );
    // TODO: Cache R_world_joint2 instead of allocating.
    Matrix3d R_world_joint2( Eigen::AngleAxisd(virtualJointPosition(3), Eigen::Vector3d::UnitZ()) *
                                          Eigen::AngleAxisd(virtualJointPosition(4), Eigen::Vector3d::UnitY()) );

    // TODO: Cache A
    Matrix3d A;
    A.col(0) = Eigen::Vector3d::UnitZ();
    A.col(1) = R_world_joint1.col(1);
    A.col(2) = R_world_joint2.col(0);

    // solve x = A.inverse() * b
    virtualJointVelocity.segment(3,3) = A.partialPivLu().solve( x_dot.bottomRows(3) );

    if (!controlit::addons::eigen::checkMagnitude(virtualJointPosition) || !controlit::addons::eigen::checkMagnitude(virtualJointVelocity))
    {
        CONTROLIT_ERROR << "Invalid virtual joint position or velocity!\n"
          << "  - virtualJointPosition: " << virtualJointPosition.transpose() << "\n"
          << "  - virtualJointVelocity: " << virtualJointVelocity.transpose() << "\n"
          << "  - x: " << x.transpose() << "\n"
          << "  - q: [" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << "]\n"
          << "  - x_dot: " << x_dot.transpose();
        return false;
    }
    return true;
}

const std::vector<std::string> & RobotState::getJointNames() const
{
    return jointNames;
}

const Vector & RobotState::getJointPosition() const
{
    return jointPosition;
}

const Vector & RobotState::getJointVelocity() const
{
    return jointVelocity;
}

const Vector & RobotState::getJointAcceleration() const
{
    return jointAcceleration;
}

const Vector & RobotState::getJointEffort() const
{
    return jointEffort;
}

const Eigen::Matrix< double, NUM_VIRTUAL_DOFS, 1 > & RobotState::getVirtualJointPosition() const
{
    return virtualJointPosition;
}

const Eigen::Matrix< double, NUM_VIRTUAL_DOFS, 1 > & RobotState::getVirtualJointVelocity() const
{
    return virtualJointVelocity;
}

} // namespace controlit
