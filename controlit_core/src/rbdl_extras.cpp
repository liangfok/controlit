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

#include <RigidBodyDynamics/Extras/rbdl_extras.hpp>
#include <rbdl/rbdl_mathutils.h>
#include <math.h>

#include <rbdl/Dynamics.h>
#include <rbdl/Kinematics.h>
#include <controlit/ControlModel.hpp>
#include <controlit/addons/eigen/LinearAlgebra.hpp>
#include <controlit/utility/ContainerUtility.hpp>

#include <controlit/logging/RealTimeLogging.hpp>

namespace RigidBodyDynamics {
namespace Extras {

Math::Vector3d calcRobotCOM(RigidBodyDynamics::Model & robot, const Math::VectorNd & Q)
{
  // Ensure the robot contains at least one body to prevent infinite loop
  // in calls between this method and the one below.
  assert(robot.mBodies.size() > 0);

  std::vector<unsigned int> linkIndexList;

  // Start at the first body that's not the root
  for(unsigned int id = 1; id < robot.mBodies.size(); id++)
  {
    linkIndexList.push_back(id);
  }

  return calcRobotCOM(robot, Q, linkIndexList);
}

Math::Vector3d calcRobotCOM(RigidBodyDynamics::Model & robot, const Math::VectorNd & Q,
  const std::vector<unsigned int> & linkIndexList)
{
  // If the provided link index list is empty, compute the
  // COM of the entire robot
  if (linkIndexList.empty()) return calcRobotCOM(robot, Q);

  Eigen::Vector4d COM, COMi;
  COM.setZero();
  COMi.setZero();
  double total_mass = 0.0;
  unsigned int id = 0;

  // Start at the first body that's not the root
  for(unsigned int ii = 0; ii < linkIndexList.size(); ii++)
  {
    id = linkIndexList[ii];
    double Mi = robot.mBodies[id].mMass;
    if (Mi > 0.0)
    {
      COMi.topRows(3) = RigidBodyDynamics::CalcBodyToBaseCoordinates(robot, Q, id,
        robot.mBodies[id].mCenterOfMass, false);
      COM.noalias() += COMi * Mi;
      total_mass += Mi;
    }
  }

  COM /= total_mass;

  return Math::Vector3d(COM.topRows(3));
}

Math::Vector3d calcRobotCOMVel(RigidBodyDynamics::Model & robot, const Math::VectorNd & Q,
  const Math::VectorNd & Qd)
{
  // Ensure the robot contains at least one body to prevent infinite loop
  // in calls between this method and the one below.
  assert(robot.mBodies.size() > 0);

  std::vector<unsigned int> linkIndexList;

  // Start at the first body that's not the root
  for(unsigned int id = 1; id < robot.mBodies.size(); id++)
  {
    linkIndexList.push_back(id);
  }

  return calcRobotCOMVel(robot, Q, Qd, linkIndexList);
}

Math::Vector3d calcRobotCOMVel(RigidBodyDynamics::Model & robot, const Math::VectorNd & Q,
  const Math::VectorNd & Qd, const std::vector<unsigned int> & linkIndexList)
{
  // If the provided link index list is empty, compute the
  // COM velocity of the entire robot
  if(linkIndexList.empty()) return calcRobotCOMVel(robot, Q, Qd);

  Eigen::Vector4d COMvel, COMveli;
  COMvel.setZero();
  COMveli.setZero();
  double total_mass = 0.0;
  unsigned int id = 0;

  //Starting at the first body that's not the root
  for(unsigned int ii = 1; ii < linkIndexList.size(); ii++)
  {
    id = linkIndexList[ii];
    double Mi = robot.mBodies[id].mMass;
    if (Mi > 0.0)
    {
      COMveli.topRows(3) = RigidBodyDynamics::CalcPointVelocity(robot, Q, Qd, id,
        robot.mBodies[id].mCenterOfMass, false);
      COMvel.noalias() += COMveli*Mi;
      total_mass += Mi;
    }
  }
  COMvel /= total_mass;

  return Math::Vector3d(COMvel.topRows(3));
}

void calcRobotJvCOM(RigidBodyDynamics::Model & robot, const Math::VectorNd & Q, Math::MatrixNd & JvCOM)
{
  // Ensure the robot contains at least one body to prevent infinite loop
  // in calls between this method and the one below.
  assert(robot.mBodies.size() > 0);

  std::vector<unsigned int> linkIndexList;

  // Start at the first body that's not the root
  for(unsigned int id = 1; id < robot.mBodies.size(); id++)
  {
    linkIndexList.push_back(id);
  }

  return calcRobotJvCOM(robot, Q, linkIndexList, JvCOM);
}

void calcRobotJvCOM(RigidBodyDynamics::Model & robot, const Math::VectorNd & Q,
  const std::vector<unsigned int> & linkIndexList, Math::MatrixNd & JvCOM)
{
  // If the provided link index list is empty, compute the
  // COM velocity of the entire robot
  if(linkIndexList.empty()) return calcRobotJvCOM(robot, Q, JvCOM);

  JvCOM.setZero(); //just to be safe
  double total_mass = 0;  //i.e., number of bodies that contribute to the mass
  Math::MatrixNd JvCOMi(3, robot.dof_count);
  unsigned int id = 0;

  //Starting at the first body that's nor the root
  for(unsigned int ii=1; ii < linkIndexList.size(); ii++)
  {
    id = linkIndexList[ii];
    double Mi = robot.mBodies[id].mMass;
    if (Mi > 0)
    {
      //calcJvCOM(robot, id, JvCOMi);
      RigidBodyDynamics::CalcPointJacobian(robot, Q, id, robot.mBodies[id].mCenterOfMass, JvCOMi, false);

      assert(JvCOM.rows() == JvCOMi.rows());
      assert(JvCOM.cols() == JvCOMi.cols());

      JvCOM += JvCOMi * Mi;
      total_mass += Mi;
    }
  }
  JvCOM /= total_mass;
}

// Calculates the rotation matrix which will rotate the first vector to the second vector
// Vectors need not be unit vectors.
//
// http://en.wikipedia.org/wiki/Rotation_matrix#Rotation_matrix_from_axis_and_angle
Math::Matrix3d calcRFromV1toV2(const Math::Vector3d & v1, const Math::Vector3d & v2)
{

  Math::Vector3d v1norm = v1; v1norm.normalize();
  Math::Vector3d v2norm = v2; v2norm.normalize();
  Math::Vector3d axis = v1norm.cross(v2norm); //axis of rotation
  double sinAngle = axis.norm(); //cosine of angle of rotation

  // This protects against the case where sinAngle > 1.
  // If sinAngle > 1, asin(sinAngle) would return NaN.
  if (sinAngle > 1) sinAngle = 1;

  double cosAngle = cos(asin(sinAngle));

  Math::Matrix3d R;
  // std::cout<<"sinAngle = "<<sinAngle<<std::endl;
  if(sinAngle < 1e-8) //The vector are aligned--or off by pi!!
  {
    if(v1norm.transpose() * v2norm > 0)
      return R.setIdentity();
    else //off by pi!
    {
      R.setIdentity();
      return -R;
    }
  }
  axis.normalize();
  /* Math::Matrix3d w;
  w.setZero();
  w(0,1) = -axis(2); w(0,2) = axis(1);
                     w(1,2) = -axis(0);
  w(1,0) = axis(2);
  w(2,0) = -axis(1); w(2,1) = axis(0);

  R = Math::Matrix3d::Identity(3,3) + w*sinAngle + w*w*(1-cosAngle);*/

  //Axis-angle to rotation matrix avoiding a few flops
  R(0, 0) = cosAngle + axis(0) * axis(0) * (1 - cosAngle);
  R(1, 1) = cosAngle + axis(1) * axis(1) * (1 - cosAngle);
  R(2, 2) = cosAngle + axis(2) * axis(2) * (1 - cosAngle);

  R(1, 0) = axis(0) * axis(1) * (1 - cosAngle) + axis(2) * sinAngle;
  R(0, 1) = axis(0) * axis(1) * (1 - cosAngle) - axis(2) * sinAngle;

  R(2, 0) = axis(0) * axis(2) * (1 - cosAngle) - axis(1) * sinAngle;
  R(0, 2) = axis(0) * axis(2) * (1 - cosAngle) + axis(1) * sinAngle;

  R(2, 1) = axis(1) * axis(2) * (1 - cosAngle) + axis(0) * sinAngle;
  R(1, 2) = axis(1) * axis(2) * (1 - cosAngle) - axis(0) * sinAngle;

  controlit::utility::ContainerUtility containerUtility;
  if (!containerUtility.checkMagnitude(R, 1e10))
  {
    CONTROLIT_ERROR_RT << "Invalid R!\n"
      << std::scientific << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 1)
      << "  - v1: " << v1.transpose() << "\n"
      << "  - v2: " << v2.transpose() << "\n"
      << "  - v1norm: " << v1norm.transpose() << "\n"
      << "  - v2norm: " << v2norm.transpose() << "\n"
      << "  - axis: " << axis.transpose() << "\n"
      << "  - sinAngle: " << sinAngle << "\n"
      << "  - cosAngle: " << cosAngle;
  }

  return R;
}

//*** NOT USED IN CODE BASE ***//
// //Calculated the full state of the body as if it were a floating body
// void calcFloatingBodyState(RigidBodyDynamics::Model& robot,
//                              unsigned int body_id,
//                              const Math::VectorNd& Q,
//                              const Math::VectorNd& Qd,
//                              Math::VectorNd& pos,
//                              Math::VectorNd& eulerYPR,
//                              Math::VectorNd& vel,
//                              Math::VectorNd& eulerYPRdot)
// {
//   //TODO: Should this be the origin of the joint frame or the center of mass?
//   //...depends on how its coded in URDF...and what Gazebo thinks.
//   pos = RigidBodyDynamics::CalcBodyToBaseCoordinates(robot,Q,
//                                              body_id,Math::Vector3d(0.0, 0.0, 0.0),
//                                              false);

//   vel = RigidBodyDynamics::CalcPointVelocity(robot,Q,Qd,
//                                              body_id,Math::Vector3d(0.0, 0.0, 0.0),
//                                              false);

//   //Need the body frame angular velocity and the rotation matrix
//   Math::SpatialVector spatialVel = robot.v[body_id];
//   Math::Matrix3d R = RigidBodyDynamics::CalcBodyWorldOrientation(robot, Q, body_id, false);
//   Math::Vector3d omega(spatialVel(0), spatialVel(1), spatialVel(2));

//   //TODO: CHECK ORDER HERE!!!!--AND CHECK eulerAngles!!!
//   Math::Vector3d eulerRPY = R.eulerAngles(0, 1, 2); //(roll, pitch, yaw).  Hopefully.
//   // Math::MatrixNd Rcheck(3,3);
//   // Rcheck = (Eigen::AngleAxisd(eulerRPY[0], Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(eulerRPY[1], Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(eulerRPY[2], Eigen::Vector3d::UnitZ())).toRotationMatrix();

//   eulerYPR(0) = eulerRPY(2);
//   eulerYPR(1) = eulerRPY(1);
//   eulerYPR(2) = eulerRPY(0);

//   // std::cout<<"R is : "<<std::endl<<R<<std::endl;
//   // std::cout<<"Rcheck is : "<<std::endl<<Rcheck<<std::endl;

//   // std::cout<<"Rcheck.transpose()*R is : "<<std::endl<<Rcheck.transpose()*R<<std::endl;


//   Math::Matrix3d Binv(3, 3); Binv.setZero();
//   Math::Vector3d eulerRPYdot;

//   //TODO: Problems with +/- pi/2?
//   double roll = eulerYPR(2);
//   double pitch =  eulerYPR(1);
//   Binv(0, 0) = 1.0;                        Binv(0,2) = -sin(pitch);
//                   Binv(1, 1) = cos(roll);  Binv(1,2) = cos(pitch) * sin(roll);
//                   Binv(2, 1) = -sin(roll); Binv(2,2) = cos(pitch) * cos(roll);
//   eulerRPYdot = Binv.inverse() * omega;
//   eulerYPRdot(0) = eulerRPYdot(2);
//   eulerYPRdot(1) = eulerRPYdot(1);
//   eulerYPRdot(2) = eulerRPYdot(0);
// }

//Functions for approximating reaction forces.
void calcRxnFrCOM(RigidBodyDynamics::Model& robot,
  const Math::VectorNd& Q,
  const Math::MatrixNd& Ainv,
  const Math::MatrixNd& U,
  const Math::VectorNd& grav,
  const std::vector<unsigned int>& body_id,
  const Math::VectorNd& Tau,
  std::vector<Math::VectorNd> & FrCOM,
  double sigmaThreshold)
{
  if(FrCOM.size() != body_id.size())
    FrCOM.clear();

  Math::MatrixNd Jcom(6 * body_id.size(), robot.dof_count);
  Math::MatrixNd Jvi(3, robot.dof_count); //Jvi.setZero();
  Math::MatrixNd Jwi(3, robot.dof_count);
  int rowct = 0;

  for(size_t ii = 0; ii < body_id.size(); ii++)
  {
    RigidBodyDynamics::CalcPointJacobian(robot, Q, body_id[ii], robot.mBodies[body_id[ii]].mCenterOfMass, Jvi, false);
    RigidBodyDynamics::CalcPointJacobianW(robot, Q, body_id[ii], robot.mBodies[body_id[ii]].mCenterOfMass, Jwi, false);

    for(int iv = 0; iv < 3; iv++)
      Jcom.row(rowct++) = Jvi.row(iv);
    for(int iw = 0; iw < 3; iw++)
      Jcom.row(rowct++) = Jwi.row(iw);
  }

  Math::MatrixNd temp = Jcom * Ainv * Jcom.transpose();
  //Math::MatrixNd lambda;
  //controlit::pseudoInverse(temp, sigmaThreshold, lambda);
  Math::MatrixNd Jsbar = Ainv * Jcom.transpose() * temp;
  Math::VectorNd fullRxn = -Jsbar.transpose() * U.transpose() * Tau + Jsbar.transpose() * grav;

  // If FrCOM is empty, append to the end of it.  Otherwise, modify it directly.
  // Note that the size check at the very beginning of this method ensures
  // that FrCOM is the expected size.
  Math::VectorNd Fri(6);
  bool empty = false;
  if(FrCOM.size() == 0)
    empty = true;

  rowct = 0;
  for(size_t ii = 0; ii < body_id.size(); ii++)
  {
    for(int ir = 0; ir < 6; ir++)
      Fri[ir] = fullRxn[rowct++];
    if(empty)
      FrCOM.push_back(Fri);
    else
      FrCOM[ii] = Fri;
  }
}

void calcRxnFr(RigidBodyDynamics::Model& robot,
  const Math::VectorNd& Q,
  const Math::MatrixNd& Ainv,
  const Math::MatrixNd& U,
  const Math::VectorNd& grav,
  const std::vector<unsigned int>& body_id,
  const std::vector<Math::VectorNd>& body_frame_point,
  const Math::VectorNd& Tau,
  std::vector<Math::VectorNd> & Fr,
  double sigmaThreshold)
{
  // CONTROLIT_DEBUG_RT << "Method Called!";

  if(Fr.size() != body_id.size())
    Fr.clear();
  Math::Vector3d pi, gi, mi, fi; //point for rxn force calculation in th world frame

  calcRxnFrCOM(robot, Q, Ainv, U, grav, body_id, Tau, Fr, sigmaThreshold);
  for(size_t ii = 0; ii < body_id.size(); ii++)
  {
    pi = RigidBodyDynamics::CalcBodyToBaseCoordinates(robot, Q, body_id[ii], body_frame_point[ii], false);
    gi = RigidBodyDynamics::CalcBodyToBaseCoordinates(robot, Q, body_id[ii],
      robot.mBodies[body_id[ii]].mCenterOfMass, false);

    for(int jj = 0; jj < 3; jj++)
    {
      fi(jj) = Fr[ii](jj);
      mi(jj) = Fr[ii](jj + 3);
    }
    mi += Math::VectorCrossMatrix(gi - pi) * fi;
    for(int jj = 0; jj < 3; jj++)
    {
      Fr[ii](jj + 3) = mi(jj);
    }
  }

  // CONTROLIT_DEBUG_RT << "Done!";
}

//Functions for calculating COP for a list of links in contact
void calcCOPfromFrCOM(RigidBodyDynamics::Model& robot,
  const Math::VectorNd& Q,
  const std::vector<unsigned int>& body_id,
  const std::vector<Math::VectorNd>& FrCOM,
  const std::vector<Math::VectorNd>& contactNormal,
  const std::vector<Math::VectorNd>& contactPlanePoint,
  std::vector<Math::VectorNd>& COP,
  const Frame& rxnFrFrame,
  const Frame& contactNormalFrame,
  const Frame& contactPlanePointFrame,
  const Frame& COPFrame)
{
  //check to see whenther COP needs to be filled in or if it's already the right size
  if(COP.size() != body_id.size())
    COP.clear();

  bool empty = false;
  if(COP.size() == 0)
    empty = true;

  //Set up utility matrixes for COP calculation
  Math::Matrix3d Id, hatMapFi, Ri, Si, SiHatMapFi;
  Id.setZero(); Id(0, 0) = 1.0; Id(1, 1) = 1.0; Id(2, 2) = 1.0;
  Math::MatrixNd LHSi(4, 3);
  Math::Vector3d f, m, fi, mi, ni, p0i, pi, gi, SiMi, SiHatMapFiG;
  Math::VectorNd rhsi(4);

  for(size_t ii = 0; ii < body_id.size(); ii++)
  {
    Ri = RigidBodyDynamics::CalcBodyWorldOrientation(robot, Q, body_id[ii], false);
    gi = RigidBodyDynamics::CalcBodyToBaseCoordinates(robot, Q, body_id[ii],
      robot.mBodies[body_id[ii]].mCenterOfMass, false); //COM location in global

    for(int jj = 0; jj < 3; jj++)
    {
      fi[jj] = FrCOM[ii][jj]; //Rxn forces on this body
      mi[jj] = FrCOM[ii][jj + 3]; //Rxn moments on this body
    }

    if(rxnFrFrame == Frame::LOCAL)//have forces in body frame, put them into global frame
    {
      fi = Ri.transpose() * fi;
      mi = Ri.transpose() * mi;
    }

    ni = contactNormal[ii];
    if(contactNormalFrame == Frame::LOCAL) //have normal in body frame, need it in world frame
      ni = Ri.transpose() * ni;

    p0i = contactPlanePoint[ii];
    if(contactPlanePointFrame == Frame::LOCAL) //have p0 in body frame, need it in world frame
      p0i = RigidBodyDynamics::CalcBodyToBaseCoordinates(robot, Q, body_id[ii], p0i, false);

    //Now that everything is in the same reference frame, compute COP in that frame
    hatMapFi = RigidBodyDynamics::Math::VectorCrossMatrix(fi);
    Si = Id - ni * ni.transpose();

    SiHatMapFi = Si * hatMapFi;
    SiHatMapFiG = SiHatMapFi * gi;
    SiMi = Si * mi;

    LHSi.row(0) = SiHatMapFi.row(0);
    LHSi.row(1) = SiHatMapFi.row(1);
    LHSi.row(2) = SiHatMapFi.row(2);
    LHSi.row(3) = ni.transpose();

    rhsi(0) = -SiMi(0) + SiHatMapFiG(0);
    rhsi(1) = -SiMi(1) + SiHatMapFiG(1);
    rhsi(2) = -SiMi(2) + SiHatMapFiG(2);
    rhsi(3) = ni.transpose() * p0i;

    Math::Matrix3d tempLHS = LHSi.transpose() * LHSi;
    pi = tempLHS.inverse() * LHSi.transpose() * rhsi;

    if(empty)
      COP.push_back(pi);
    else
      COP[ii] = pi;

    if(COPFrame==Frame::LOCAL) //if we want the COP in the in the body frame, put it these.
      COP[ii] = RigidBodyDynamics::CalcBaseToBodyCoordinates(robot, Q, body_id[ii], pi, false);
  }
}

void calcCOPfromFr(RigidBodyDynamics::Model& robot,
  const Math::VectorNd& Q,
  const std::vector<unsigned int>& body_id,
  const std::vector<Math::VectorNd>& body_frame_point,
  const std::vector<Math::VectorNd>& Fr,
  const std::vector<Math::VectorNd>& contactNormal,
  const std::vector<Math::VectorNd>& contactPlanePoint,
  std::vector<Math::VectorNd>& COP,
  const Frame& rxnFrFrame,
  const Frame& contactNormalFrame,
  const Frame& contactPlanePointFrame,
  const Frame& COPFrame)
{

  //check to see whenther COP needs to be filled in or if it's already the right size
  if(COP.size() != body_id.size())
    COP.clear();

  bool empty = false;
  if(COP.size() == 0)
    empty = true;

  //All actual calculations will be done in the body frame.
  bool calcRi = true;
  if(rxnFrFrame == Frame::LOCAL && contactNormalFrame == Frame::LOCAL)
    calcRi = false;

  //Set up utility functions
  Math::Matrix3d Id, hatMapFi, Ri, Si, SiHatMapFi;
  Id.setZero(); Id(0, 0) = 1.0; Id(1, 1) = 1.0; Id(2, 2) = 1.0;
  Math::MatrixNd LHSi(4, 3);
  Math::Vector3d f, m, fi, mi, ni, p0i, pi, gi, SiMi, SiHatMapFiG;
  Math::VectorNd rhsi(4);

  for(size_t ii = 0; ii < body_id.size(); ii++)
  {
    gi = body_frame_point[ii]; //COM location in body frame
    if(calcRi)
      Ri = RigidBodyDynamics::CalcBodyWorldOrientation(robot, Q, body_id[ii], false);

    for(int jj = 0; jj < 3; jj++)
    {
      f[jj] = Fr[ii][jj]; //Rxn forces on this body
      m[jj] = Fr[ii][jj + 3]; //Rxn moments on this body
    }

    if(rxnFrFrame == Frame::WORLD)
    {
      fi = Ri * f;
      mi = Ri * m;
    }
    else //already in local body frame
    {
      fi = f;
      mi = m;
    }

    if(contactNormalFrame == Frame::WORLD)
      ni = Ri * contactNormal[ii];
    else //already in local body frame
      ni = contactNormal[ii];

    if(contactPlanePointFrame == Frame::WORLD) //have p0 in world frame, need it in body frame
      p0i = RigidBodyDynamics::CalcBaseToBodyCoordinates(robot, Q, body_id[ii], contactPlanePoint[ii], false);
    else //already in local body frame
      p0i = contactPlanePoint[ii];

    //Now that everything is in the same reference frame, compute COP in that frame
    hatMapFi = RigidBodyDynamics::Math::VectorCrossMatrix(fi);
    Si = Id - ni * ni.transpose();

    //std::cout<<"Si is : "<<std::endl<<Si<<std::endl;

    SiHatMapFi = Si * hatMapFi;
    SiHatMapFiG = SiHatMapFi * gi;
    SiMi = Si * mi;

    LHSi.row(0) = SiHatMapFi.row(0);
    LHSi.row(1) = SiHatMapFi.row(1);
    LHSi.row(2) = SiHatMapFi.row(2);
    LHSi.row(3) = ni.transpose();

    rhsi(0) = -SiMi(0) + SiHatMapFiG(0);
    rhsi(1) = -SiMi(1) + SiHatMapFiG(1);
    rhsi(2) = -SiMi(2) + SiHatMapFiG(2);
    rhsi(3) = ni.transpose() * p0i;


    Math::Matrix3d tempLHS = LHSi.transpose() * LHSi;
    pi = tempLHS.inverse() * LHSi.transpose() * rhsi;

    if(empty)
      COP.push_back(pi);
    else
      COP[ii] = pi;

    if(COPFrame==Frame::WORLD) //if we want the COP in the world frame, put it there.
      COP[ii] = RigidBodyDynamics::CalcBodyToBaseCoordinates(robot, Q, body_id[ii], pi, false);
  }
}

//Functions for calculating COP one link at a time
void calcCOPfromFrCOM(RigidBodyDynamics::Model& robot,
  const Math::VectorNd& Q,
  const unsigned int& body_id,
  const Math::VectorNd& FrCOM,
  const Math::VectorNd& contactNormal,
  const Math::VectorNd& contactPlanePoint,
  Math::VectorNd& COP,
  const Frame& rxnFrFrame,
  const Frame& contactNormalFrame,
  const Frame& contactPlanePointFrame,
  const Frame& COPFrame)
{
  //All actual calculations will be done in the WORLD frame.
  //Set up utility matrixes for COP calculation
  Math::Matrix3d Id, hatMapFi, Ri, Si, SiHatMapFi;
  Id.setZero(); Id(0, 0) = 1.0; Id(1, 1) = 1.0; Id(2, 2) = 1.0;
  Math::MatrixNd LHSi(4,3);
  Math::Vector3d f, m, fi, mi, ni, p0i, pi, gi, SiMi, SiHatMapFiG;
  Math::VectorNd rhsi(4);

  //TODO: make this an input when done testing!
  //double M = robot.mBodies[body_id].mMass/2;

  Ri = RigidBodyDynamics::CalcBodyWorldOrientation(robot, Q, body_id, false); //Ri rotates vectors from WORLD to BODY frame
  gi = RigidBodyDynamics::CalcBodyToBaseCoordinates(robot, Q, body_id, robot.mBodies[body_id].mCenterOfMass, false); //COM location in global frame

  for(int jj = 0; jj < 3; jj++)
  {
    fi[jj] = FrCOM[jj]; //Rxn forces on this body
    mi[jj] = FrCOM[jj + 3]; //Rxn moments on this body
  }

  if(rxnFrFrame == Frame::LOCAL)//have forces in body frame, put them into global frame
  {
    fi = Ri.transpose() * fi;
    mi = Ri.transpose() * mi;
  }

  //fi -= M*robot.gravity;

  ni = contactNormal;
  if(contactNormalFrame == Frame::LOCAL) //have normal in body frame, need it in world frame
    ni = Ri.transpose() * ni;

  p0i = contactPlanePoint;
  if(contactPlanePointFrame == Frame::LOCAL) //have p0 in body frame, need it in world frame
    p0i = RigidBodyDynamics::CalcBodyToBaseCoordinates(robot, Q, body_id, p0i, false);

  //Now that everything is in the same reference frame, compute COP in that frame
  hatMapFi = RigidBodyDynamics::Math::VectorCrossMatrix(fi);
  Si = Id - ni * ni.transpose();

  SiHatMapFi = Si * hatMapFi;
  SiHatMapFiG = SiHatMapFi * gi;
  SiMi = Si * mi;

  LHSi.row(0) = SiHatMapFi.row(0);
  LHSi.row(1) = SiHatMapFi.row(1);
  LHSi.row(2) = SiHatMapFi.row(2);
  LHSi.row(3) = ni.transpose();

  rhsi(0) = -SiMi(0) + SiHatMapFiG(0);
  rhsi(1) = -SiMi(1) + SiHatMapFiG(1);
  rhsi(2) = -SiMi(2) + SiHatMapFiG(2);
  rhsi(3) = ni.transpose() * p0i;

  Math::Matrix3d tempLHS = LHSi.transpose() * LHSi;
  pi = tempLHS.inverse() * LHSi.transpose() * rhsi;

  /*check equivialent solution.....*/
  Math::MatrixNd LHSch(3, 3), I23;  I23.setZero(2, 3);
  I23(0, 0) = 1; I23(1, 1) = 1;
  Math::VectorNd rhsch(3);

  Math::Matrix3d RzTon;
  RzTon = RigidBodyDynamics::Extras::calcRFromV1toV2(Eigen::Vector3d::UnitZ(), ni);
  LHSch.topRows(2) = I23 * RzTon.transpose() * SiHatMapFi;
  LHSch.bottomRows(1) = ni.transpose();
  rhsch.head(2) = I23 * RzTon.transpose() * (-SiMi + SiHatMapFiG);
  rhsch.tail(1) = ni.transpose() * p0i;
  // Math::Vector3d COPch = LHSch.inverse()*rhsch;

  //std::cout<<"supposed cop = "<<pi.transpose()<<std::endl;
  //std::cout<<"check cop = "<<COPch.transpose()<<std::endl;

  /*std::cout<<"Si*mi+SiHatMapFi*pi-SiHatMapFiG : \n"<<Si*mi+SiHatMapFi*pi-SiHatMapFiG<<std::endl;
  std::cout<<"mi+HatMapFi*pi-HatMapFiG : \n"<<mi+hatMapFi*pi-hatMapFi*gi<<std::endl;*/

  COP = pi;

  if(COPFrame == Frame::LOCAL) //if we want the COP in the in the body frame, put it these.
    COP = RigidBodyDynamics::CalcBaseToBodyCoordinates(robot, Q, body_id, pi, false);
}

void calcCOPfromFr(RigidBodyDynamics::Model& robot,
  const Math::VectorNd& Q,
  const unsigned int& body_id,
  const Math::VectorNd& body_frame_point,
  const Math::VectorNd& Fr,
  const Math::VectorNd& contactNormal,
  const Math::VectorNd& contactPlanePoint,
  Math::VectorNd& COP,
  const Frame& rxnFrFrame,
  const Frame& contactNormalFrame,
  const Frame& contactPlanePointFrame,
  const Frame& COPFrame)
{
  //All actual calculations will be done in the BODY frame.
  bool calcRi = true;
  if(rxnFrFrame == Frame::LOCAL && contactNormalFrame == Frame::LOCAL)
    calcRi = false;

  //Set up utility functions
  Math::Matrix3d Id, hatMapFi, Ri, Si, SiHatMapFi;
  Id.setZero(); Id(0, 0) = 1.0; Id(1, 1) = 1.0; Id(2, 2) = 1.0;
  Math::MatrixNd LHSi(4, 3);
  Math::Vector3d f, m, fi, mi, ni, p0i, pi, gi, SiMi, SiHatMapFiG;
  Math::VectorNd rhsi(4);

  gi = body_frame_point; //COM location in body frame
  if(calcRi)
    Ri = RigidBodyDynamics::CalcBodyWorldOrientation(robot, Q, body_id, false);

  for(int jj = 0; jj < 3; jj++)
  {
    f[jj] = Fr[jj]; //Rxn forces on this body
    m[jj] = Fr[jj + 3]; //Rxn moments on this body
  }

  if(rxnFrFrame == Frame::WORLD)
  {
    fi = Ri * f;
    mi = Ri * m;
  }
  else // already in local body frame
  {
    fi = f;
    mi = m;
  }

  if(contactNormalFrame == Frame::WORLD)
    ni = Ri * contactNormal;
  else //already in local body frame
    ni = contactNormal;

  if(contactPlanePointFrame == Frame::WORLD) //have p0 in world frame, need it in body frame
    p0i = RigidBodyDynamics::CalcBaseToBodyCoordinates(robot, Q, body_id, contactPlanePoint, false);
  else //already in local body frame
    p0i = contactPlanePoint;

  //Now that everything is in the same reference frame, compute COP in that frame
  hatMapFi = RigidBodyDynamics::Math::VectorCrossMatrix(fi);
  Si = Id - ni * ni.transpose();

  //std::cout<<"Si is : "<<std::endl<<Si<<std::endl;

  SiHatMapFi = Si * hatMapFi;
  SiHatMapFiG = SiHatMapFi * gi;
  SiMi = Si * mi;

  LHSi.row(0) = SiHatMapFi.row(0);
  LHSi.row(1) = SiHatMapFi.row(1);
  LHSi.row(2) = SiHatMapFi.row(2);
  LHSi.row(3) = ni.transpose();

  rhsi(0) = -SiMi(0) + SiHatMapFiG(0);
  rhsi(1) = -SiMi(1) + SiHatMapFiG(1);
  rhsi(2) = -SiMi(2) + SiHatMapFiG(2);
  rhsi(3) = ni.transpose() * p0i;


  Math::Matrix3d tempLHS = LHSi.transpose() * LHSi;
  pi = tempLHS.inverse() * LHSi.transpose() * rhsi;

  if(COPFrame == Frame::LOCAL)
    COP = pi;

  if(COPFrame == Frame::WORLD) //if we want the COP in the world frame, put it there.
    COP = RigidBodyDynamics::CalcBodyToBaseCoordinates(robot, Q, body_id, pi, false);
}

// Dynamics
/*
void calcQdd(ControlModel& robot, Vector & Qdd, const Vector & Tau, std::vector<SpatialVector> * f_ext = NULL)
{
  RigidBodyDynamics::ForwardDynamics(robot.rbdlModel(), robot.getQ(), robot.getQd(), Tau, Qdd, f_ext);
}

void calcTau(ControlModel& robot, Vector & Tau, std::vector<SpatialVector> * f_ext = NULL)
{
  RigidBodyDynamics::InverseDynamics(robot.rbdlModel(), robot.getQ(), robot.getQd(), robot.getQdd(), Tau, f_ext);
}
*/

} // namespace Extras
} // namespace RigidBodyDynamics
