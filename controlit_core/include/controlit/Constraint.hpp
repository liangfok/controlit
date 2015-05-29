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

#ifndef __CONTROLIT_CORE_CONSTRAINT_HPP__
#define __CONTROLIT_CORE_CONSTRAINT_HPP__

#include <string>
#include <rbdl/rbdl.h>
#include <yaml-cpp/yaml.h>

#include <controlit/PlanElement.hpp>

namespace controlit {

/*!
 * The top-level class of all constraints.
 */
class Constraint : public PlanElement
{
protected:

    /*!
     * The default constructor.
     */
    Constraint();

    /*!
     * The constructor.
     *
     * \param type The type name.
     * \param name The instance name.
     */
    Constraint(std::string const& type, std::string const& name);

public:

    /*!
     * Loads a task configuration from a YAML specification.
     *
     * \param node The YAML node containing the specification.
     */
    void loadConfig(YAML::Node const& node);

    /*!
     * Saves the constraint configuration into a YAML specification.
     *
     * \param node The node in which to save the specification.
     */
    void saveConfig(YAML::Emitter& node) const;

    /*!
     * Initializes this constraint.
     *
     * \param[in] robot The robot model.
     */
    virtual void init(RigidBodyDynamics::Model& robot);

    /*!
     * Determines whether this constraint is initialized.
     *
     * \return true if the constraint is initialized.
     */
    bool isInitialized() const;

    /*!
     * For internal constraints, the master node is the "independent" node.
     * For example, Dreamer's torso has two joints that are connected by a belt
     * that forces their positions to always be the same.  In this case, the
     * actuated joint is the "master" while the joint that is connected by a belt
     * is the "slave".
     *
     * For external constraints (e.g., contact constraints with the outside world),
     * the master node is the only node that is constrained
     *
     * \return the name of the master node in internal constraints and the only
     * node for external constraints.
     */
    virtual std::string getMasterNodeName() { return masterNodeName_; }

    /*!
     * Internal constraints may constrain more than one joint.
     * This method returns the number of joints that are constrained.
     *
     * \return the number of DOFs that are constrained.
     */
    inline unsigned int getNConstrainedDOFs() { return constrainedDOFs_; }

    /*!
     * Returns the ID of the master node.  This is the "independent" node in internal
     * constraints, and the only node in external constraints.
     *
     * \return The ID of the master node.
     */
    unsigned int getMasterNode() const;

    /*!
     * Returns true if there is a "dependent" node in internal constraints.
     *
     * \return Whether there is a slave node in the constraint.
     */
    bool hasSlaveNode() const;

    /*!
     * Returns the ID of the slave node in the constraint.
     *
     * TODO: Support multiple slave nodes!
     *
     * \return The ID of the slave node.
     */
    unsigned int getSlaveNode() const;

    /*!
     * Returns true if this constraint is a contact constraint.  This is used
     * by the controller to determine whether the constraint should be included
     * in internal tension control calculations.
     *
     * \return true if this is a contact constraint.
     */
    bool isContactConstraint() const {return isContactConstraint_;}

    /*!
     * An accessor for the constraint's Jacobian matrix.
     * It needs to be defined by child classes.
     *
     * \param[in] robot the robot model.
     * \param[in] Q The current joint state of the robot.
     * \param[out] Jc The constraint's Jacobian matrix.
     */
    virtual void getJacobian(RigidBodyDynamics::Model& robot, const Vector& Q, Matrix& Jc) = 0;

protected:

    /*!
     * Declares the parameters of this constraint.
     */
    virtual void setupParameters();

    /*!
     * Whether this constraint is initialized.
     */
    bool initialized_;

    /*!
     * Whether this constraint is a contact constraint.
     */
    bool isContactConstraint_;

    /*!
     * The number of DOFs that are being constrainted.
     *
     * QUESTION: will this ever be a value other than 1 or 2?
     */
    unsigned int constrainedDOFs_;

    /*!
     * The name of the master node that is being constrained.
     */
    std::string masterNodeName_;

    /*!
     * The name of the slave node.  This is only used for internal
     * constraints.
     */
    std::string slaveNodeName_;

    /*!
     * The ID of the master node that is being constrained.
     */
    unsigned int masterNode_;

    /*!
     * The ID of the slave node.  Note that this is only used for
     * internal constraints.
     */
    unsigned int slaveNode_;

    /*!
     * Our local copy of the constraint Jacobian.
     */
    Matrix localJc;

    /*!
     * A pointer to the local copy of the constraint Jacobian matrix.
     */
    Parameter * localJcParam;
};

} // namespace controlit

#endif // __CONTROLIT_CORE_CONSTRAINT_HPP__
