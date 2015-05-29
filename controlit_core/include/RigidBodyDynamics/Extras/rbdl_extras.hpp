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

#ifndef __CONTROLIT_MODEL_RBDL_EXTRAS_HPP__
#define __CONTROLIT_MODEL_RBDL_EXTRAS_HPP__

// TODO: Move this file to controlit_addons_rbdl

#include <rbdl/rbdl.h>
#include <controlit/addons/eigen/LinearAlgebra.hpp>
#include <vector>

namespace controlit{
    class ControlModel;
}

namespace RigidBodyDynamics {
namespace Extras {

enum Frame {
    WORLD = 0,
    LOCAL = 1
};

// KINEMATICS OVERLOADS
/*!
* Computes the Jacobian (Jv) that converts the velocities of all joints in the robot (Qdot)
* to the velocity of a particular point on the robot.  The point is given with respect to the
* reference frame of a body with the specified id.
*
* The velocity of the specified point is Jv * Qdot.
*
* \param robot The robot model.
* \param body_id The ID of the body whose coordinate frame is used to define the point.
* \param point The point whose Jacobian is being computed.
* \param Jv The resulting Jacobian matrix.
*/
/*static void calcJv(model::ControlModel& robot, unsigned int body_id, const Vector3d point, Matrix & Jv);
//! Jacobian (Jv) of the center of mass of a body
static void calcJvCOM(model::ControlModel& robot, unsigned int body_id, Matrix & Jv);
//! Jacobian (Jw) of a point expressed in body coordinates
static void calcJw(model::ControlModel& robot, unsigned int body_id, const Vector3d point, Matrix & Jv);
//! 3d rotation of the body's frame
static Matrix3d calcR(model::ControlModel& robot, unsigned int body_id);
//! Calculates body COM position in base frame
static Vector3d calcPointPos(model::ControlModel& robot, unsigned body_id, Vector3d body_frame_point);
//! Calculates body COM position in base frame
static Vector3d calcCOMPos(model::ControlModel& robot, unsigned body_id);
//! Calculates body COM velocity in base frame
static Vector3d calcCOMVel(model::ControlModel& robot, unsigned body_id);
//! Calculate robot COM
*/

/*!
 * Computes the location of the robot's COM.
 *
 * \param[in] robot The robot model.
 * \param[in] Q The current genereralized joint positions
 * \return The COM location.
 */
Math::Vector3d calcRobotCOM(RigidBodyDynamics::Model & robot, const Math::VectorNd & Q);

/*!
 * Computes the location of the COM of a subset of the robot's links, as specified by their index
 *
 * \param[in] robot The robot model.
 * \param[in] Q The current genereralized joint positions
 * \param[in] linkIndexList  The list of IDs of the links that should be used in the COM calculation.
 * If this list is empty, the COM of the entire robot will be used.
 * \return The COM location.
 */
Math::Vector3d calcRobotCOM(RigidBodyDynamics::Model & robot, const Math::VectorNd & Q,
    const std::vector<unsigned int> & linkIndexList);

/*!
 * Computes the velocity of the robot's COM.
 *
 * \param[in] robot The robot model.
 * \param[in] Q The current genereralized joint positions
 * \param[in] Qd The current generalized joint velocities
 * \return The COM velocity.
 */
Math::Vector3d calcRobotCOMVel(RigidBodyDynamics::Model & robot, const Math::VectorNd & Q,
    const Math::VectorNd & Qd);

/*!
 * Computes the location of the COM velocity of a subset of the robot's links, as specified by their index
 *
 * \param[in] robot The robot model.
 * \param[in] Q The current genereralized joint positions
 * \param[in] Qd The current generalized joint velocities
 * \param[in] linkIndexList  The list of IDs of the links that should be used in the COM velocity calculation.
 * If this list is empty, the COM velocity of the entire robot will be used.
 * \return The COM velocity.
 */
Math::Vector3d calcRobotCOMVel(RigidBodyDynamics::Model & robot, const Math::VectorNd & Q,
    const Math::VectorNd & Qd, const std::vector<unsigned int> & linkIndexList);

/*!
 * Computes the Jacobian of the robot's COM.
 *
 * \param[in] robot The robot model.
 * \param[in] Q The current genereralized joint positions
 * \param[out] JvCOM A reference to where the resulting Jacobian should be stored.
 */
void calcRobotJvCOM(RigidBodyDynamics::Model & robot, const Math::VectorNd & Q, Math::MatrixNd & JvCOM);

/*!
 * Computes the Jacobian of the COM of a subset of the robot's links.
 *
 * \param[in] robot The robot model.
 * \param[in] Q The current genereralized joint positions
 * \param[in] linkIndexList  The list of IDs of the links that should be used in the COM velocity calculation.
 * If this list is empty, the COM velocity of the entire robot will be used.
 * \param[out] JvCOM A reference to where the resulting Jacobian should be stored.
 */
void calcRobotJvCOM(RigidBodyDynamics::Model & robot, const Math::VectorNd & Q,
    const std::vector<unsigned int> & linkIndexList, Math::MatrixNd & JvCOM);

/*!
 * Calculates the rotation matrix that will rotate the first vector into the second vector
 * The vectors need not be unit vectors.
 *
 * \param[in] v1 The first vector.
 * \param[in] v2 The second vector.
 * \return A rotation matrix.
 */
Math::Matrix3d calcRFromV1toV2(const Math::Vector3d & v1, const Math::Vector3d & v2);

//*** NOT USED IN CODE BASE ***//
// //Calculates the world-frame state of the specified body as if it were a floating body
// //attached to the world by the following joint.  In this joint the angular state
// //are given by the yaw-pitch-roll euler angles and their rates of change.
// rbdl_joint = Joint (
//              SpatialVector (0., 0., 0., 1., 0., 0.),
//              SpatialVector (0., 0., 0., 0., 1., 0.),
//              SpatialVector (0., 0., 0., 0., 0., 1.),
//              SpatialVector (0., 0., 1., 0., 0., 0.),
//              SpatialVector (0., 1., 0., 0., 0., 0.),
//              SpatialVector (1., 0., 0., 0., 0., 0.));
// void calcFloatingBodyState(RigidBodyDynamics::Model& robot,
//                                                   unsigned int body_id,
//                                                   const Math::VectorNd& Q,
//                                                   const Math::VectorNd& Qd,
//                                                   Math::VectorNd& pos,
//                                                   Math::VectorNd& eulerYPR,
//                                                   Math::VectorNd& vel,
//                                                   Math::VectorNd& eulerYPRdot);

// DYNAMICS OVERLOADS
//! Forward dynamics--accelereations due to applied forces
//static void calcQdd(model::ControlModel& robot, Vector & Qdd, Vector const& Tau, std::vector<SpatialVector> * f_ext = NULL);
//! Inverse dynamics--due to external forces
//static void calcTau(model::ControlModel& robot, Vector & Tau, std::vector<SpatialVector>* f_ext = NULL);

// Functions for approximating reaction forces due to an arbitrary number of contacts

/*!
 * Computes the expected reaction forces at the COM of the bodies in contact with the
 * environment based on a robot model.
 *
 * \param[in] robot The robot model.
 *
 * \param[in] Q The generalized joint positions.  The length should be # DOFs.
 *
 * \param[in] Ainv The inverse of the joint-space inertial matrix.
 * Its dimesions should be: # DOFs x # DOFs.
 *
 * \param[in] U The underactuation matrix.  Its dimensions are # actuable DOFs x # DOFs.
 * Each row contains information about a particular actuable joint.  It is all zeros except
 * for a one in the column that corresponds to the same joint in the RBDL model.
 *
 * \param[in] grav The joint space gravity vector, 'grav'.  This is
 * the amount of force due to gravity that each joint feels.
 * The length of this vector is #DOFs, which is the number of DOFs in the RBDL model and
 * includes both real and virtual joints.
 *
 * \param[in] body_id A list of body IDs of the bodies that are in contact.  The length
 * of this list should be between one and # actuable DOFs.
 *
 * \param[in] Tau The command issued by the controller.  Its length is # actuable DOFs.
 *
 * \param[out] FrCOM The reaction forces.  This is where the results are stored.
 * Its length is equal to the size of parameter body_id.
 */
void calcRxnFrCOM(RigidBodyDynamics::Model& robot,
    const Math::VectorNd& Q,
    const Math::MatrixNd& Ainv,
    const Math::MatrixNd& U,
    const Math::VectorNd& grav,
    const std::vector<unsigned int>& body_id,
    const Math::VectorNd& Tau,
    std::vector<Math::VectorNd> & FrCOM,
    double sigmaThreshold = 0.0001);

/*!
 * Computes the expected reaction forces at an arbitrary point on the bodies in contact
 * with the environment based on a robot model.
 *
 * \param[in] robot The robot model.
 *
 * \param[in] Q The generalized joint positions.  The length should be # DOFs.
 *
 * \param[in] Ainv The inverse of the joint-space inertial matrix.
 * Its dimesions should be: # DOFs x # DOFs.
 *
 * \param[in] U The underactuation matrix.  Its dimensions are # actuable DOFs x # DOFs.
 * Each row contains information about a particular actuable joint.  It is all zeros except
 * for a one in the column that corresponds to the same joint in the RBDL model.
 *
 * \param[in] grav The joint space gravity vector, 'grav'.  This is
 * the amount of force due to gravity that each joint feels.
 * The length of this vector is #DOFs, which is the number of DOFs in the RBDL model and
 * includes both real and virtual joints.
 *
 * \param[in] body_id A list of body IDs of the bodies that are in contact.  The length
 * of this list should be between one and # actuable DOFs.
 *
 * \param[in] body_frame_point A list of points in the bodies' coordinate frames where the
 * reaction forces should be computed.  Its length should be equal to that of parameter
 * body_id.
 *
 * \param[in] Tau The command issued by the controller.  Its length is # actuable DOFs.
 *
 * \param[out] Fr The reaction forces.  This is where the results are stored.
 * Its length is equal to the size of parameter body_id.
 */
void calcRxnFr(RigidBodyDynamics::Model& robot,
    const Math::VectorNd& Q,
    const Math::MatrixNd& Ainv,
    const Math::MatrixNd& U,
    const Math::VectorNd& grav,
    const std::vector<unsigned int>& body_id,
    const std::vector<Math::VectorNd>& body_frame_point,
    const Math::VectorNd& Tau,
    std::vector<Math::VectorNd> & Fr,
    double sigmaThreshold = 0.0001);

//Functions for calculating COP for a list of links in contact

/*!
 * Calculates the COPs for a set of links in contact with the environment based on the
 * reaction forces at COMs of the links.
 *
 * \param[in] robot The robot model.
 *
 * \param[in] Q The generalized joint positions.
 *
 * \param[in] body_id A list of body IDs of the bodies that are in contact with the
 * environment.  The length of this list should be between one and # actuable DOFs.
 *
 * \param[in] FrCOM The reaction force at the COMs of the links.
 *
 * \param[in] contactNormal The contact normal vectors.
 *
 * \param[in] contactPlanePoint Points on the contact planes.
 *
 * \param[out] COP The COP of the links in contact with the environment.
 *
 * \param[in] rxnFrFrame The frame in which the reaction forces are defined.
 *
 * \param[in] contactNormalFrame The frame in which the contact normals are defined.
 *
 * \param[in] contactPlanePointFrame The frame in which the contact point is defined.
 *
 * \param[in] COPFrame The frame in which the COP is defined.
 */
void calcCOPfromFrCOM(RigidBodyDynamics::Model& robot,
    const Math::VectorNd& Q,
    const std::vector<unsigned int>& body_id,
    const std::vector<Math::VectorNd>& FrCOM,
    const std::vector<Math::VectorNd>& contactNormal,
    const std::vector<Math::VectorNd>& contactPlanePoint,
    std::vector<Math::VectorNd>& COP,
    const Frame& rxnFrFrame  = Frame::WORLD,
    const Frame& contactNormalFrame = Frame::WORLD,
    const Frame& contactPlanePointFrame = Frame::WORLD,
    const Frame& COPFrame = Frame::LOCAL);

/*!
 * Calculates the COPs for a set of links in contact with the environment based on the
 * reaction forces at points on the links.
 *
 * \param[in] robot The robot model.
 *
 * \param[in] Q The generalized joint positions.
 *
 * \param[in] body_id A list of body IDs of the bodies that are in contact with the
 * environment.  The length of this list should be between one and # actuable DOFs.
 *
 * \param[in] body_frame_point A list of points in the bodies' coordinate frames where the
 * reaction forces are given.  Its length should be equal to that of parameter
 * body_id.
 *
 * \param[in] Fr The reaction forces at the body_frame_point points.
 *
 * \param[in] contactNormal The contact normal vectors.
 *
 * \param[in] contactPlanePoint Points on the contact planes.
 *
 * \param[out] COP The COP of the links in contact with the environment.
 *
 * \param[in] rxnFrFrame The frame in which the reaction forces are defined.
 *
 * \param[in] contactNormalFrame The frame in which the contact normals are defined.
 *
 * \param[in] contactPlanePointFrame The frame in which the contact point is defined.
 *
 * \param[in] COPFrame The frame in which the COP is defined.
 */
void calcCOPfromFr(RigidBodyDynamics::Model& robot,
    const Math::VectorNd& Q,
    const std::vector<unsigned int>& body_id,
    const std::vector<Math::VectorNd>& body_frame_point,
    const std::vector<Math::VectorNd>& Fr,
    const std::vector<Math::VectorNd>& contactNormal,
    const std::vector<Math::VectorNd>& contactPlanePoint,
    std::vector<Math::VectorNd>& COP,
    const Frame& rxnFrFrame  = Frame::WORLD,
    const Frame& contactNormalFrame = Frame::WORLD,
    const Frame& contactPlanePointFrame = Frame::WORLD,
    const Frame& COPFrame = Frame::LOCAL);


//Functions for calculating center of pressure for one link at a time

/*!
 * Calculates the COP for a link in contact with the environment based on the
 * reaction force at the COM of the link.
 *
 * \param[in] robot The robot model.
 *
 * \param[in] Q The generalized joint positions.
 *
 * \param[in] body_id The body ID of the body in contact with the environment.
 *
 * \param[in] FrCOM The reaction force at the COM point on the link.
 *
 * \param[in] contactNormal The contact normal vector.
 *
 * \param[in] contactPlanePoint A point on the contact plane.
 *
 * \param[out] COP The COP of the links in contact with the environment.
 *
 * \param[in] rxnFrFrame The frame in which the reaction forces are defined.
 *
 * \param[in] contactNormalFrame The frame in which the contact normals are defined.
 *
 * \param[in] contactPlanePointFrame The frame in which the contact point is defined.
 *
 * \param[in] COPFrame The frame in which the COP is defined.
 */
void calcCOPfromFrCOM(RigidBodyDynamics::Model& robot,
    const Math::VectorNd& Q,
    const unsigned int& body_id,
    const Math::VectorNd& FrCOM,
    const Math::VectorNd& contactNormal,
    const Math::VectorNd& contactPlanePoint,
    Math::VectorNd& COP,
    const Frame& rxnFrFrame  = Frame::WORLD,
    const Frame& contactNormalFrame = Frame::WORLD,
    const Frame& contactPlanePointFrame = Frame::WORLD,
    const Frame& COPFrame = Frame::LOCAL);

/*!
 * Calculates the COP for a link in contact with the environment based on the
 * reaction force at a point on the link.
 *
 * \param[in] robot The robot model.
 *
 * \param[in] Q The generalized joint positions.
 *
 * \param[in] body_id The body ID of the body in contact with the environment.
 *
 * \param[in] body_frame_point A point in the body's coordinate frames where the
 * reaction force is given.
 *
 * \param[in] Fr The reaction force at the point on the link.
 *
 * \param[in] contactNormal The contact normal vector.
 *
 * \param[in] contactPlanePoint A point on the contact plane.
 *
 * \param[out] COP The COP of the links in contact with the environment.
 *
 * \param[in] rxnFrFrame The frame in which the reaction forces are defined.
 *
 * \param[in] contactNormalFrame The frame in which the contact normals are defined.
 *
 * \param[in] contactPlanePointFrame The frame in which the contact point is defined.
 *
 * \param[in] COPFrame The frame in which the COP is defined.
 */
void calcCOPfromFr(RigidBodyDynamics::Model& robot,
    const Math::VectorNd& Q,
    const unsigned int & body_id,
    const Math::VectorNd& body_frame_point,
    const Math::VectorNd& Fr,
    const Math::VectorNd& contactNormal,
    const Math::VectorNd& contactPlanePoint,
    Math::VectorNd& COP,
    const Frame& rxnFrFrame  = Frame::WORLD,
    const Frame& contactNormalFrame = Frame::WORLD,
    const Frame& contactPlanePointFrame = Frame::WORLD,
    const Frame& COPFrame = Frame::LOCAL);
} // namespace Extras
} // namespace RigidBodyDynamics

#endif
