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

#ifndef __CONTROLIT_CORE_VIRTUAL_LINKAGE_MODEL_HPP__
#define __CONTROLIT_CORE_VIRTUAL_LINKAGE_MODEL_HPP__

#include <vector>
#include <yaml-cpp/yaml.h>
#include <rbdl/rbdl.h>

#include <controlit/ReflectionRegistry.hpp>
#include <controlit/addons/eigen/LinearAlgebra.hpp>
#include <controlit/ConstraintSet.hpp>
#include <controlit/utility/ContainerUtility.hpp>

//======================================
//  - CLASS DEFINITION -
/*!
  \brief Container class for constraint objects
 */
//======================================
namespace controlit {

//class RigidBodyDynamics::Model;
class ConstraintSet;

// Virtual Linkage model stores matrices and does calculations related to internal tension control
class VirtualLinkageModel
{
public:
    /*!
     * The default constructor.
     */
    VirtualLinkageModel();

    /*!
     * The constructor.
     *
     * \param name The name of this constraint set.
     */
    VirtualLinkageModel(std::string name);

    /*!
     * The destructor.
     */
    virtual ~VirtualLinkageModel();

    //Commented out for now, fully defined by constraint specification
    /*bool loadConfig(YAML::Node const& node);
    bool saveConfig(YAML::Emitter& node) const;*/

    /*!
     * Initializes the virual linkage model matrices.  If necessary, this will also
     * initialize the constraint set.
     *
     * \param[in] model The RBDL model.
     * \param[in] constraintSet The constraint set.
     */
    void init(RigidBodyDynamics::Model& model, ConstraintSet& constraintSet);

    /*!
     * Updates Wint, Jc, JcBar, Nc, UNc, UNcBar, Lstar--Assumes that the ConstraintSet HAS already been updated
     */
    void update(RigidBodyDynamics::Model& robot, ConstraintSet& updatedConstraints, const Vector& Q, Matrix const& Ainv);

    /*!
     * Determines whether it is valid to do calculations with this VLM
     */
    bool exists() const {return exists_;}

    /*!
     * Retruns the threshold for singular values
     */
    double getSigmaThreshold() const {return sigmaThreshold_;}

     /*!
     * Returns number of contacts
     */
    int getContactCount() const {return contactConstraintCount_;}

    /*!
     * Gets Wint, the internal part of the grasp matrix for Fr at COP
     *
     * \param Wint, the internal part of the grasp matrix for Fr at COP
     */
    void getWint(Matrix& Wint) const;

    /*!
     * Gets Wint, the internal part of the grasp matrix for Fr at COP
     *
     * \return Wint, the internal part of the grasp matrix for Fr at COP
     */
    const Matrix& getWint() const {return Wint_;}

    /*!
     * Gets WintSensor, the internal part of the grasp matrix for Fr at SENSOR location
     *
     * \param WintSensor, the internal part of the grasp matrix for Fr at SENSOR location
     */
    void getWintSensor(Matrix& WintSensor) const;

    /*!
     * Gets WintSensor, the internal part of the grasp matrix for Fr at SENSOR location
     *
     * \return WintSensor, the internal part of the grasp matrix for Fr at SENSOR location
     */
    const Matrix& getWintSensor() const {return WintSensor_;}

    /*TODO: Determine if these functions will be useful and implement them...*/
    /*!
     * Gets Phi, the map from minimal dof reaction forces to full (redundant) reaction wrench in world frame.
     *
     * \param Phi, the map from minimal dof reaction forces to full (redundant) reaction wrench in world frame.
     */
    void getPhi(Matrix& Phi);

    /*!
     * Gets Phi, the map from minimal dof reaction forces to full (redundant) reaction wrench in world frame.
     *
     * \return Phi, the map from minimal dof reaction forces to full (redundant) reaction wrench in world frame.
     */
    const Matrix& getPhi() const {return Phi_;}

    /*!
     * Gets Jc, the Jacobian of the contact constraints
     *
     * \param Jc, the Jacobian of the contact constraints.
     */
    void getJacobian(Matrix& Jc) const;

    /*!
     * Gets Jc, the Jacobian of the contact constraints
     *
     * \return Jc, the Jacobian of the contact constraints.
     */
    const Matrix& getJacobian() const {return Jc_;}

    /*!
     * Gets Jc Bar, the dynamically consistent pseudo-inverse of Jc.
     *
     * \param JcBar This is where the result is stored.
     */
    void getJacobianBar(Matrix& JcBar) const;

    /*!
     * Gets Jc Bar, the dynamically consistent pseudo-inverse of Jc.
     *
     * \return JcBar This is where the result is stored.
     */
    const Matrix& getJacobianBar() const {return JcBar_;}

     /*!
     * Gets U, the underactuation matrix
     *
     * \param U This is where the result is stored.
     */
    void getU(Matrix& U) const;

    /*!
     * Gets U, the underactuation matrix
     *
     * \return U This is where the result is stored.
     */
    const Matrix& getU() const {return U_;}

     /*!
     * Gets Nc, the nullspace of the constraints that accounts for
     * underacutation (?)
     *
     * \param UNc This is where the result is stored.
     */
    void getNc(Matrix& Nc) const;

    /*!
     * Gets Nc, the nullspace of the constraints that accounts for
     * underacutation (?)
     *
     * \return Nc This is where the result is stored.
     */
    const Matrix& getNc() const {return Nc_;}

    /*!
     * Gets UNc, the nullspace of the constraints that accounts for
     * underacutation (?)
     *
     * \param UNc This is where the result is stored.
     */
    void getUNc(Matrix& UNc) const;

    /*!
     * Gets UNc, the nullspace of the constraints that accounts for
     * underacutation (?)
     *
     * \return UNc This is where the result is stored.
     */
    const Matrix& getUNc() const {return UNc_;}

    /*!
     * Gets UNcBar, the dynamically consistent pseudo-inverse of UNc
     *
     * \param UNcBar This is where the result is stored.
     */
    void getUNcBar(Matrix& UNcBar) const;

    /*!
     * Gets UNcBar, the dynamically consistent pseudo-inverse of UNc
     *
     * \return UNcBar This is where the result is stored.
     */
    const Matrix& getUNcBar() const {return UNcBar_;}

    /*!
     * Gets Lstar, the projection into the constact space
     *
     * \param Lstar This is where the result is stored.
     */
    void getLstar(Matrix& Lstar) const;

    /*!
     * Gets Lstar, the projection into the constact space
     *
     * \retturn Lstar This is where the result is stored.
     */
    const Matrix& getLstar() const {return Lstar_;}

    /*!
     * Gets FrSensor, the concatenated force sensor info in global frame
     *
     * \param FrSensor This is where the result is stored.
     */
    void getFrSensor(Vector& FrSensor) const;

    /*!
     * Gets FrSensor, the concatenated force sensor info in global frame
     *
     * \retturn FrSensor This is where the result is stored.
     */
    const Vector& getFrSensor() const {return FrSensor_;}

private:
      /*!
       * Defines the threshold above which a value is considered to be infinity.
       * This is used in checking the validity of various data structures.
       */
      #define INFINITY_THRESHOLD 1e10

     /*!
     * Updates the contact constraint Jacobian by picking rows from
     * JcFull_
     *
     * \return Whether the update was successful.
     */
    bool updateJc(ConstraintSet& constraints);

     /*!
     * Updates Wint_ and (by default) Phi_
     */
    void updateWint(RigidBodyDynamics::Model& robot, ConstraintSet& constraints, const Vector& Q);

    /*!
     * Sets up and resizes local matrices.
     *
     * \param[in] dof The total number of DOFs in the model (both real and virtual)
     * \param[in] consDOFcount The number of constrainted DOFs.
     * \param[in] unactDOFcount The number of unactuated DOFs.
     * \param[in] virtualDOFcount The number of virtual DOFs.
     */
    void resize(size_t dof, size_t consDOFcount, size_t unactDOFcount, size_t virtualDOFcount);

    /*!
     * Determines whether the set of enabled constraints have changed.
     *
     * \param updatedConstraints The constraints.
     * \return true if the set of enabled constraints have changed.
     */
    bool enableSetChanged(ConstraintSet& updatedConstraints);

    /*!
     * Constraint set which describes the virtual linkage model
     */
    std::vector<size_t> contactConstraintRowIndices_; //row indices of constratinSet_->getJacobian() due to external contact constraints
    std::vector<size_t> contactConstraintIndices_; //indices of constraintSet_->getConstraintSet() which correspond to contact constraints
    std::vector<std::pair<size_t, size_t>> nrowsAndStartRow; //the number of rows in the i^th jc, and the starting row in the concantenated minimal DOF fr
    std::vector<std::pair<size_t, size_t>> indexPairs_; //all unique pairs of contacts
    size_t contactConstraintCount_; //number of contacts
    size_t pairCount_; //number of pairs of contacts
    size_t contactConstraintRows_; //Number of rows in JcFull_ due to contact constraints--a.k.a, number of rows in Jc_

    /*!
     * Potentially interesting values to be stored and returned as necessary.
     */
    bool initialized_;
    bool exists_; //Use to check whether this is even valid
    double sigmaThreshold_; //For SVD--singular values
    Matrix Wint_; //Internal part of grasp matrix for Fr at COP
    Matrix WintSensor_; //Internal part of grasp matrix for Fr at Sensor location
    Matrix Phi_; //block-matrix concatenation of phi_
    Matrix U_; //CHECK--should this include transmission constraints(?)--YES, it should!!!
    Matrix Jc_; //Jacobian of constact constraint set
    Matrix JcBar_; //Dynamically consistent psuedo-inverse of Jc_;
    Matrix Nc_; //Id_col - JcBar_*Jc_
    Matrix UNc_; //U_*Nc_
    Matrix UNcBar_; //Dynamically consistent psuedo-inverse of U*Nc
    Matrix Lstar_; //Id_{gamma} - UNc_*UNcBar_
    Vector FrSensor_; //Convenient to store 6*contactConstraintCount Fr_ in the right order here, since ContactConstraints are hooked up to contact sensors

    /*!
     * Utility variables for internal calulcations
     */
    Matrix JcFull_; //Full Jacobian of constraints including transmission constraints
    Matrix Id_col;  //Identity with size of columns in Jc_
    Matrix Id_row;  //Identity with size of rows in Jc_
    Matrix Id_gamma; //Identity with size of rows in U_
    Matrix RtDeltaOperator_; //R_t*\delta_t; rotates forces to directions uniting COPs and sorts forces to top
    Matrix Ts_; //block-matrix transformation to local surface frames with z-normal convention and sorts moments to bottom
    Matrix TsSensor_;
    Matrix selectionMatrixFt_;
    Matrix selectionMatrixMn_;
    Matrix selectionMatrixMt_;

    /*!
     * Utility variables to keep track of stuff for the i^th contact
     */
    std::vector<size_t> bodyId_;
    //std::vector<Vector> deltaCOP_; //Probably don't need to store this(!!)
    std::vector<Vector> worldCOPs_; //(Goal) COP locations in WORLD frame
    std::vector<Vector> contactSensorLocations_; //Location of contact sensor (in LOCAL frame)
    std::vector<Vector> contactNormals_; //Normal to contact plane (in LOCAL frame)
    std::vector<Matrix> phiMaps_;

    /*!
     * Records the enable state of each constraint in the virtual
     * linkage model.
     */
    std::vector<bool> enableState_;

      /*!
       * Ensures the matrices used in the controller's computations remain valid.
       */
      controlit::utility::ContainerUtility containerUtility;
};

} // namespace controlit

#endif  // __CONTROLIT_CORE_VIRTUAL_LINKAGE_MODEL_HPP__
