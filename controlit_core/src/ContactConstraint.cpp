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
#include <controlit/ContactConstraint.hpp>
#include <controlit/parser/yaml_parser.hpp>

namespace controlit {

ContactConstraint::ContactConstraint(std::string const& type, std::string const& name) :
    Constraint(type,name),
    sensorLocation_(3)
{
    isContactConstraint_ = true;
}

ContactConstraint::ContactConstraint() :
    Constraint("__UNDEFINED_TYPE__", "__UNDEFINED_NAME__"),
    sensorLocation_(3)
{
    isContactConstraint_ = true;
}

void ContactConstraint::init(RigidBodyDynamics::Model& robot)
{
    //Parent class init sets masterNode_'s value, call it first
    Constraint::init(robot);

    //Sets up any parameters which are optional input.
    if(sensorLocation_.size()!=3)
    {
        sensorLocation_.resize(3);
        sensorLocation_ = robot.mBodies[masterNode_].mCenterOfMass;
    }
}

void ContactConstraint::setupParameters()
{
    CONTROLIT_PR_DEBUG << "Method called.";

    // Set up parent parameters
    Constraint::setupParameters();

    // Now declare my parameters
    declareParameter("goalLocalCOP", &goalLocalCOP_);
    declareParameter("goalWorldCOP", &goalWorldCOP_);
    declareParameter("localCOP", &localCOP_);
    declareParameter("worldCOP", &worldCOP_);
    declareParameter("rxnForceCOM", &rxnForceCOM_); //SHOULD acutally be at the sensorLocation_ (?), could just leave it and require the input the be effective rxn force at COM
    declareParameter("contactNormal", &contactNormal_);
    declareParameter("contactPlanePoint", &contactPlanePoint_);
    declareParameter("reactionForceFrame", &rxnFrFrame_);
    declareParameter("goalCOPFrame", &goalCOPFrame_);
    declareParameter("sensorLocation", &sensorLocation_);
    declareParameter("contactFrame",&contactFrame_);

    paramLocalCOP = this->lookupParameter("localCOP");
    paramWorldCOP = this->lookupParameter("worldCOP");
    paramGoalLocalCOP = this->lookupParameter("goalLocalCOP");
    paramGoalWorldCOP = this->lookupParameter("goalWorldCOP");
}

void ContactConstraint::publishParameters()
{
    paramLocalCOP->set(localCOP_);
    paramWorldCOP->set(worldCOP_);
    paramGoalLocalCOP->set(goalLocalCOP_);
    paramGoalWorldCOP->set(goalWorldCOP_);
}

} // namespace controlit
