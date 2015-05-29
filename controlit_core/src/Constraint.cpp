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

#include <controlit/logging/RealTimeLogging.hpp>
#include <controlit/Constraint.hpp>
#include <controlit/parser/yaml_parser.hpp>

//======================================
//  - CLASS DEFINITION -rosmak
/*!
  \brief The class is fully implemented in this header.
 */
//======================================
namespace controlit {

Constraint::Constraint() :
    PlanElement("Constraint", "__UNDEFINED_CONSTRAINT_NAME__"),
    initialized_(false),
    isContactConstraint_(false),
    constrainedDOFs_(0),
    masterNodeName_(""),
    slaveNodeName_(""),
    masterNode_(std::numeric_limits<unsigned int>::max()),
    slaveNode_(std::numeric_limits<unsigned int>::max()),
    localJcParam(nullptr)
{
}

Constraint::Constraint(std::string const& typeName, std::string const& instanceName) :
    PlanElement(typeName, instanceName),
    initialized_(false),
    isContactConstraint_(false),
    constrainedDOFs_(0),
    masterNodeName_(""),
    slaveNodeName_(""),
    masterNode_(std::numeric_limits<unsigned int>::max()),
    slaveNode_(std::numeric_limits<unsigned int>::max()),
    localJcParam(nullptr)
{
}

void Constraint::loadConfig(YAML::Node const& node)
{
    node["type"] >> typeName;
    node["name"] >> instanceName;

    parse_parameter_reflection(node, static_cast<ParameterReflection*>(this));  //At runtime, this will load all subclass parameters

    // CONTROLIT_PR_INFO << "Load config complete";
}

void Constraint::saveConfig(YAML::Emitter& node) const
{
    emit_parameter_reflection(node, static_cast<ParameterReflection const*>(this));
    // CONTROLIT_PR_INFO << "Save config complete";
}

void Constraint::init(RigidBodyDynamics::Model& robot)
{
    //assert(!isInitialized());

    masterNode_ = robot.GetBodyId(masterNodeName_.c_str());
    if (masterNode_ == std::numeric_limits<unsigned int>::max())
    {
        std::stringstream sstr;
        sstr << "init: master body " << masterNodeName_ << " not found! Aborting initialization. Valid body names: " << std::endl;

        // Get valid body names
        for (auto const& it : robot.mBodyNameMap)
        {
            sstr << it.first << "(" << it.second << ")" << std::endl;
        }

        CONTROLIT_PR_ERROR << sstr.str();

        return;
    }

    if (!slaveNodeName_.empty())
    {
        slaveNode_ = robot.GetBodyId(slaveNodeName_.c_str());
        if (slaveNode_ == std::numeric_limits<unsigned int>::max())
        {
            std::stringstream sstr;
            sstr << "init: slave body " << slaveNodeName_ << " not found! Aborting initialization. Valid body names: " << std::endl;

            // Get valid body names
            for (auto const& it : robot.mBodyNameMap)
            {
                sstr << it.first << "(" << it.second << ")" << std::endl;
            }

            CONTROLIT_PR_ERROR << sstr.str();

            return;
        }
    }

    // Get a pointer to the local constraint Jacobian.  This is used to publish
    // the bound constraint jacobian parameter.
    localJcParam = lookupParameter("constraintJacobian");

    initialized_ = true;

    // CONTROLIT_PR_INFO << "Initialization complete";
}

bool Constraint::isInitialized() const
{
    return initialized_;
}

//! "Independent" node in internal constraints, only node in external constraints
unsigned int Constraint::getMasterNode() const
{
    assert(isInitialized());
    return masterNode_;
}

bool Constraint::hasSlaveNode() const
{
    assert(isInitialized());
    return (slaveNode_ != std::numeric_limits<unsigned int>::max());
}

//! "Dependent" node in internal constraints, NULL for external constraints
unsigned int Constraint::getSlaveNode() const
{
    assert(isInitialized());
    return slaveNode_;
}

void Constraint::setupParameters()
{
    declareParameter("masterNodeName", & masterNodeName_);
    declareParameter("constraintJacobian", & localJc);
}

} // namespace controlit
