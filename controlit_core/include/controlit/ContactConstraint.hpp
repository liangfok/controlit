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

#ifndef __CONTROLIT_MODEL_ContactConstraint_HPP__
#define __CONTROLIT_MODEL_ContactConstraint_HPP__

#include <string>
#include <rbdl/rbdl.h>

#include <controlit/Constraint.hpp>
#include <controlit/addons/eigen/LinearAlgebra.hpp>


namespace controlit {

using controlit::addons::eigen::Vector;
using controlit::addons::eigen::Matrix3d;

// All ContactConstraints must extend this
class ContactConstraint : public Constraint
{
protected:
  //! Constructors
  ContactConstraint(std::string const& type, std::string const& name);
  ContactConstraint();

public:
  virtual Vector getWorldCOP() const {return worldCOP_;}
  virtual Vector getLocalCOP() const {return localCOP_;}
  virtual Vector getGoalWorldCOP() const {return goalWorldCOP_;}
  virtual Vector getGoalLocalCOP() const {return goalLocalCOP_;}
  virtual Vector getRxnForceCOM() const {return rxnForceCOM_;}
  virtual Vector getContactNormal() const {return contactNormal_;}
  virtual Vector getContactPlanePoint() const {return contactPlanePoint_;}
  virtual Vector getSensorLocation() const {return sensorLocation_;}
  virtual int getRxnFrFrame() const {return rxnFrFrame_;}
  virtual int goalCOPFrame() const {return goalCOPFrame_;}
  virtual int contactFrame() const {return contactFrame_;}

  /*!
   * Recomputes \phi : R^{constrainedDOFs_} \rightarrow R^6 for the given contact constraint.
   *
   * \param Rn, Rotation from surface normal to x-axis in world frame (Rn * Rb^T * \hat{contactNormal_} = \hat{x})
   * \param Rb, Rotation from WORLD/base frame coordinates to LOCAL/body frame coordinates
   */
  virtual Matrix getPhi(const Matrix3d& Rn, const Matrix3d& Rb) = 0;
  virtual Matrix getPhi() const {return phi_;}

protected:
  void init(RigidBodyDynamics::Model& robot);

  virtual void setupParameters();

  /*PARAMETERS*/

  //! INPUT desired center of pressure in LOCAL frame or CALCULATED from goalWorldCOP_ (depending on goalCOPFrame_)
  Vector goalLocalCOP_; //

  //! CALCULATED FROM goalLocalCOP_ or INPUT desired (depending on goalCOPFrame_)
  Vector goalWorldCOP_; //

  //! CALCULATED center of pressure in LOCAL frame from rxnForceCOM_
  Vector localCOP_; //formerly cp1_ or "contactPoint"

  //! CALCULATED center of pressure in WORLD frame from rxnForceCOM_
  Vector worldCOP_;

  //! SENSED 6 DOF Reaction Force at Center of Mass (now in WORLD Frame from Gazebo sensor)
  Vector rxnForceCOM_;

  //! SENSED/INPUT 3 DOF Normal to contact plane in LOCAL (body) frame
  Vector contactNormal_;

  //! SENSED/INPUT 3 DOF Point in the contact plane in the LOCAL (body) frame
  Vector contactPlanePoint_;

  //! INPUT 3DOF LOCAL (body) frame sensor location (COM for now)
  Vector sensorLocation_;

  //! INPUT Reaction force frame (0 = world, 1 = local) (i.e., expected input from sensor data)
  int rxnFrFrame_;

  //! INPUT Reaction force frame (0 = world, 1 = local) (i.e., if goal commands will be set in the WORLD or LOCAL frame)
  int goalCOPFrame_;

  //! INPUT contact frame (0 = world, 1 = local) (i.e., if contact normal and contact plane point are specified in the WORLD or LOCAL frame)
  int contactFrame_;

  /*NOT PARAMETERS...well possibly could be a parameter....I suppose*/
  //! Mapping from minimal DOF( = constrainedDOFs_) reaction to full 6 DOF reaction wrench in WORLD Frame
  Matrix phi_;

  //Pointers to avoid constantly looking up parameters
  controlit::Parameter * paramLocalCOP;
  controlit::Parameter * paramWorldCOP;
  controlit::Parameter * paramGoalLocalCOP;
  controlit::Parameter * paramGoalWorldCOP;

  void publishParameters();

};

}

#endif
