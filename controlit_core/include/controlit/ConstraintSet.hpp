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

#ifndef __CONTROLIT_CORE_CONSTRAINT_SET_HPP__
#define __CONTROLIT_CORE_CONSTRAINT_SET_HPP__

#include <vector>
#include <yaml-cpp/yaml.h>
#include <rbdl/rbdl.h>

#include <controlit/ReflectionRegistry.hpp>
#include <controlit/addons/eigen/LinearAlgebra.hpp>

namespace controlit {

class Constraint;
class ConstraintFactory;

/*!
 * A container class for constraint objects.
 */
class ConstraintSet : public ReflectionRegistry
{
public:
    typedef std::vector< std::shared_ptr<Constraint> > ConstraintList_t;
    typedef std::vector<int> ActuatedJointIds_t;
  
    /*!
     * The default constructor.
     */
    ConstraintSet();
  
    /*!
     * The constructor.
     *
     * \param name The name of this constraint set.
     */
    ConstraintSet(std::string name);
  
    /*!
     * The destructor.
     */
    virtual ~ConstraintSet();
  
    /*!
     * Loads the constraint set from a YAML specification.
     *
     * \param[in] node The YAML node containing the constraint specifications.
     */
    bool loadConfig(YAML::Node const & node);
  
    /*!
     * Saves the constraint set into a YAML specification.
     *
     * \param[in] node The YAML node to which to save the constraint specifications.
     */
    bool saveConfig(YAML::Emitter & node) const;
  
    /*!
     * Initializes this constraint set.  This should be called after the
     * constraints are added (using loadConfig(...)) and must be re-called
     * if the enabled state of any of the constraints change.
     *
     * \param[in] robot The robot model.
     */
    bool init(RigidBodyDynamics::Model & robot);
  
    /*!
     * Updates Jc, JcBar, Nc, UNc, UNcBar
     *
     * \param[in] robot The RBDL mode.
     * \param[in] Q The current joint positions.
     * \param[in] Ainv The current Ainv.
     */
    void update(RigidBodyDynamics::Model & robot, const Vector & Q, const Matrix & Ainv);
  
    /*!
     * Gets the constraint set.
     *
     * \return the immuatable constraint set
     */
    ConstraintList_t const& getConstraintSet();
  
    /*!
     * Adds a constraint to this constraint set.
     *
     * \param[in] constraint A pointer to the constraint to add.
     */
    void addConstraint(Constraint * constraint);
  
    /*!
     * Gets whether a particular joint is constrained.
     *
     * \param[in] id The ID of the joint to check.
     * \return true if the joint is constrainted.
     */
    // bool isConstrained(unsigned int id) const;
  
    /*!
     * Gets whether this constraint set is initialized.
     *
     * \return true if the constraint set is initialized.
     */
    bool isInitialized() const { return initialized_; }
  
    /*!
     * Gets the indices of the actuable joints.
     *
     * \return The actuable joint indices.
     */
    ActuatedJointIds_t const & getActuatedJointIndices() const;
  
    /*!
     * Gets the total number of enabled constraints.
     * Note that some constraints may limit multiple DOFs.
     *
     * \return the number of enabled constraints
     */
    size_t getNConstraints() const;
  
    /*!
     * Gets U, the underactuation matrix.  This matrix specifies which DOFs are actuable.
     * Its dimensions are: # actuable DOFs x # DOFs
     *
     * \param[out] U A reference to where the underactution matrix should be stored.
     */
    void getU(Matrix& U) const;
  
    /*!
     * Gets U, the underactuation matrix.  This matrix specifies which DOFs are actuable.
     * Its dimensions are: # actuable DOFs x # DOFs
     *
     * \return The underactution matrix U.
     */
    const Matrix& getU() const {return U_;}
  
  
    /*!
     * Gets virtualU, the underactuation matrix accounting only for virtual joints.
     * This matrix specifies which DOFs are real (but not necessarily actuable).
     * Its dimensions are: # real DOFs x # DOFs
     *
     * \param[out] virtualU A reference to where the underactution matrix should be stored.
     */
    void getVirtualU(Matrix& virtualU) const;
  
    /*!
     * Gets virtualU, the underactuation matrix accounting only for virtual joints.
     * This matrix specifies which DOFs are real (but not necessarily actuable).
     * Its dimensions are: # real DOFs x # DOFs
     *
     * \return The underactution matrix virtualU.
     */
    const Matrix& getVirtualU() const {return virtualU_;}
  
    /*!
     * Gets Jc, the Jacobian matrix of the constraints.
     * Its dimensions are dependent on whether there are any constraints
     * in this ConstraintSet.
     *
     * If there are constraints, the dimensions of Jc are:
     * # constrained DOFs x # DOFs.
     *
     * If there are no constraints, the dimsions of Jc are:
     * # DOFs x # DOFs, and is in fact an identity matrix.
     *
     * \param[out] Jc, the Jacobian matrix of the contact constraints.
     */
    void getJacobian(Matrix& Jc) const;
  
    /*!
     * Gets Jc, the Jacobian matrix of the constraints.
     * Its dimensions are dependent on whether there are any constraints
     * in this ConstraintSet.
     *
     * If there are constraints, the dimensions of Jc are:
     * # constrained DOFs x # DOFs.
     *
     * If there are no constraints, the dimsions of Jc are:
     * # DOFs x # DOFs, and is in fact an identity matrix.
     *
     * \return The Jacobian matrix of the contact constraints.
     */
    const Matrix& getJacobian() const {return Jc_;}
  
    /*!
     * Gets JcBar, the dynamically consistent pseudo-inverse of Jc.
     * Its dimensions are opposite of Jc.
     *
     * \param[out] JcBar A reference to where the result is stored.
     */
    void getJacobianBar(Matrix& JcBar) const;
  
    /*!
     * Gets JcBar, the dynamically consistent pseudo-inverse of Jc.
     * Its dimensions are opposite of Jc.
     *
     * \return JcBar This is where the result is stored.
     */
    const Matrix& getJacobianBar() const {return JcBar_;}
  
    /*!
     * Gets Nc, the nullspace of the constraint set.  Its dimensions
     * are # DOFs x # DOFs.
     *
     * \param[out] A reference to where Nc should be stored.
     */
    void getNc(Matrix& Nc) const;
  
    /*!
     * Gets Nc, the nullspace of the constraint set.  Its dimensions
     * are # DOFs x # DOFs.
     *
     * \return The nullspace of the constraint set.
     */
    const Matrix& getNc() const {return Nc_;}
  
    /*!
     * Gets UNc, the product of the underactuation matrix multiplied
     * by the nullspace of the constraint set.  Another way of saying
     * this is UNc is the nullspace of the constraint set projected by
     * the underactuation matrix.
     *
     * The dimensions of UNc are # actuable DOFs x # DOFs.
     *
     * \return The nullspace of the constraint set projected by
     * the underactuation matrix.
     */
    void getUNc(Matrix& UNc) const;
  
    /*!
     * Gets UNc, the product of the underactuation matrix multiplied
     * by the nullspace of the constraint set.  Another way of saying
     * this is UNc is the nullspace of the constraint set projected by
     * the underactuation matrix.
     *
     * The dimensions of UNc are # actuable DOFs x # DOFs.
     *
     * \return The nullspace of the constraint set projected by
     * the underactuation matrix.
     */
    const Matrix& getUNc() const {return UNc_;}
  
    /*!
     * Gets UNcAiNorm, the Ai-norm of UNc_
     *
     * \param UNcAiNorm This is where the result is stored.
     */
    void getUNcAiNorm(Matrix& UNcAiNorm) const;
  
    /*!
     * Gets UNcAiNorm, tha Ai-norm of UNc_
     *
     * \return UNcAiNorm This is where the result is stored.
     */
    const Matrix& getUNcAiNorm() const {return UNcAiNorm_;}
  
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
  
  
    //returns real but unactuated dofs (i.e., slave joints)
    unsigned int getNumUnactuatedDof() const;
  
    unsigned int getNumVirtualDof() const;
  
    unsigned int getNConstrainedDOFs() const;
  
    double getSigmaThreshold() const {return sigmaThreshold_;}
  
    /*!
     * Dumps the state of this sensor set to an output stream.
     *
     * \param os The output stream.
     * \param prefix A prefix that is included at the beginning of each line
     * in the text dump.
     */
    void dump(std::ostream& os, std::string const& prefix) const;
  
private:
    /*!
     * Recomputes the constraint Jacobian.
     *
     * \param robot The robot model.
     */
    void updateJc(RigidBodyDynamics::Model& robot, const Vector& Q);
  
    /*!
     * Resizes member matrix variables.
     *
     * \param[in] dof The total number of DOFs in the robot (real and virtual)
     * \param[in] consDOFcount The number of constrained DOFs.
     * \param[in] unactDOFcount The number of unactuated DOFs.
     * \param[in] virtualDOFcount The number of virutal DOFs.  This is typically 6
     * (3 rotation and 3 prismatic joints).
     */
    void resize(unsigned int dof, unsigned int consDOFcount,
        unsigned int unactDOFcount, unsigned int virtualDOFcount);
  
    /*!
     * Whether init(...) was called.
     */
    bool initialized_;
  
    /*!
     * For SVD--singular values.
     */
    double sigmaThreshold_;
  
    /*!
     * The number of constrained DOFs.
     * This is the number of rows in Jc_.
     */
    unsigned int consDOFcount_;
  
    /*!
     * The number of unactuated DOFs among the real joints.
     */
    unsigned int unactDOFcount_;
  
    /*!
     * The number of virtual DOFs.  This is typically 6.
     */
    unsigned int virtualDOFcount_;
  
    /*!
     * The underactuation matrix for entire constraint set.
     * Its dimensions are: (# actuable DOFs x # DOFs)
     */
    Matrix U_;
  
  
    /*!
     * The underactuation matrix which selects for all *REAL*
     * joints, whether they are slave joints or not.
     * Its dimensions are: (# real DOFs x # DOFs)
     */
    Matrix virtualU_;
  
    /*!
     * The concatenated Jacobian of the constraints in this constraint set.
     * Its dimensions are: (# constrained DOFs x # DOFs)
     */
    Matrix Jc_;
  
    /*!
     * Dynamically consistent psuedo-inverse of Jc_.
     * Its dimensions are: (# DOFs x # constrained DOFs)
     */
    Matrix JcBar_;
  
    /*!
     * The nullspace of the constraint set (???)
     * Equation:  Nc_ = Id_col - JcBar_*Jc_
     */
    Matrix Nc_;
  
    /*!
     * The nullspace of the constraint set for the actuable DOFs (???)
     * Equation: UNc_ = U_ * Nc_
     */
    Matrix UNc_;
  
    /*!
     * The Ai norm of Unc. (???)
     * Equation: UNcAiNorm_ = UNc_*Ai_*UNc_.transpose()
     */
    Matrix UNcAiNorm_;
  
    /*!
     * Dynamically consistent psuedo-inverse of U*Nc (???)
     */
    Matrix UNcBar_;
  
    /*!
     * Identity matrix with size = # columns in Jc_.
     */
    Matrix Id_col;
  
    /*!
     * Identity matrix with size = # rows in Jc_.
     */
    Matrix Id_row;
  
    /*!
     * The set of constraints.
     */
    ConstraintList_t constraintSet_;
  
    /*!
     * A pointer to the contraint factory.  This factory
     * creates constraints based on YAML specifications.
     */
    std::unique_ptr<ConstraintFactory> constraintFactory;
  
    /*!
     * A list of joint indices of actuable joints.
     */
    ActuatedJointIds_t actuatedJointIndices;
  
    /*!
     * Determines whether the set of enabled constraints have changed
     * since init(...) was last called.
     *
     * \return true if the set of enabled constraints have changed.
     */
    bool enableSetChanged();
  
    /*!
     * This holds the enabled state of the constraints at the time when init(..) was
     * called. It is used by method enableSetChanged().
     */
    std::vector<int> enableState_;
};

} // namespace controlit

#endif  // __CONTROLIT_CORE_CONSTRAINT_SET_HPP__
