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

#include <boost/range/irange.hpp>

#include <controlit/ConstraintFactory.hpp>
#include <controlit/addons/eigen/LinearAlgebra.hpp>
#include <controlit/addons/eigen/PseudoInverse.hpp>
#include <controlit/parser/yaml_parser.hpp>
#include <controlit/ConstraintSet.hpp>

/*!
 * A container class for constraits.
 */
namespace controlit {

// Uncomment the appropriate line below for enabling/disabling the printing of debug statements
#define PRINT_DEBUG_STATEMENT(ss)
// #define PRINT_DEBUG_STATEMENT(ss) CONTROLIT_PR_DEBUG << ss;

#define PRINT_DEBUG_STATEMENT_RT(ss)
// #define PRINT_DEBUG_STATEMENT_RT(ss) CONTROLIT_PR_DEBUG_RT << ss;

ConstraintSet::ConstraintSet() :
    ReflectionRegistry("constraint_set","__UNAMED__"),
    initialized_(false),
    sigmaThreshold_(0.0001),
    consDOFcount_(0),
    unactDOFcount_(0),
    virtualDOFcount_(0)
{
    constraintFactory.reset(new ConstraintFactory());
}

ConstraintSet::ConstraintSet(std::string name) :
    ReflectionRegistry("constraint_set", name),
    initialized_(false),
    sigmaThreshold_(0.0001),
    consDOFcount_(0),
    unactDOFcount_(0),
    virtualDOFcount_(0)
{
    constraintFactory.reset(new ConstraintFactory());
}

ConstraintSet::~ConstraintSet()
{
    constraintSet_.clear(); // This is to prevent segfault when an instance of this class is destroyed.
    ReflectionRegistry::clearParameterCollection();
}

bool ConstraintSet::loadConfig(YAML::Node const& doc)
{
    PRINT_DEBUG_STATEMENT("Method called!");

    // Before anything, load all the constraints in the 'constraints' section.
    // We will come back later and query the ConstraintFactory for the constraints
    // we need.
    std::map<std::string, Constraint*> constraintList;

    YAML::Node const* constraintsNode = doc.FindValue("constraints"); // TODO: Move YAML tag names to a class
    if (constraintsNode != NULL)
    {
        for (YAML::Iterator it = constraintsNode->begin(); it != constraintsNode->end(); ++it)
        {
            Constraint* constraint = constraintFactory->loadFromYaml(*it);
            if (constraint != NULL)
                constraintList.insert(std::make_pair(constraint->getInstanceName(), constraint));
            else
            {
                CONTROLIT_PR_WARN << "Factory failed to create constraint";
            }
        }
    }
    else
    {
        CONTROLIT_PR_WARN << "'constraints' section in YAML config not found. No constraints will be loaded.";
    }

    // Load Constraint Set parameters
    YAML::Node const* csNode = doc.FindValue("constraint_set"); // TODO: Move YAML tag names to a class
    if (csNode == NULL)
    {
        CONTROLIT_PR_ERROR << "'constraint_set' section in YAML config not found. Aborting load config.";
        return false;
    }
    (*csNode)["type"] >> typeName;
    (*csNode)["name"] >> instanceName;

    PRINT_DEBUG_STATEMENT("type: " << typeName << ", name: " << instanceName);

    // Load the active constraint list
    YAML::Node const* activeConstraintsNode = csNode->FindValue("active_constraints"); // TODO: Move YAML tag names to a class
    if (activeConstraintsNode != NULL)
    {
        for (YAML::Iterator it = activeConstraintsNode->begin(); it != activeConstraintsNode->end(); ++it)
        {
            std::string name;
            int enabled;

            (*it)["name"] >> name;
            if ((*it).FindValue("enabled"))
            {
                (*it)["enabled"] >> enabled;
            }
            else
            {
                PRINT_DEBUG_STATEMENT("No enabled specification found.  Assuming enabled = true.")
                enabled = 1;
            }

            auto tuple = constraintList.find(name);
            if (tuple != constraintList.end())
            {
                tuple->second->lookupParameter("enabled")->set(enabled);
                addConstraint(tuple->second);
            }
            else
            {
                CONTROLIT_PR_WARN << "Requested constraint " << name << " in 'active_constraints' "
                            << "section but no constraint of that name was present in 'constraints' section. Typo?";
            }
        }
    }
    else
    {
        CONTROLIT_PR_WARN << "'active_constraints' section in YAML config not found. "
                    << "Loaded constraints will not be associated with this constraint set.";
    }
    // Load the rest of my parameters (none now for constraints)
    parse_parameter_reflection(*csNode, static_cast<ParameterReflection*>(this));

    PRINT_DEBUG_STATEMENT("load config complete")

    return true;
}

bool ConstraintSet::saveConfig(YAML::Emitter& node) const
{
    PRINT_DEBUG_STATEMENT("save config complete")
    return true;
}

bool ConstraintSet::init(RigidBodyDynamics::Model & robot)
{
    PRINT_DEBUG_STATEMENT("Method called!")

    // Initialize the number of virtual DOFs to be zero
    virtualDOFcount_ = 0;

    // Determine the number of virtual DOFs by iterating through
    // the list of links in the RBDL model and determining the
    // index of the first link with a mass greater than zero.
    for (unsigned int id = 1; id < robot.dof_count; id++)
    {
        if (robot.mBodies[id].mMass > 0)
        {
            virtualDOFcount_ = id;
            break;
        }
    }

    PRINT_DEBUG_STATEMENT("Number of virtual DOFs: " << virtualDOFcount_)

    // Add the remaining body IDs to actuatedJointIndices.
    // The constrained joints will be removed after the constraints
    // are initialized (see below).
    actuatedJointIndices.clear();
    for (int id = virtualDOFcount_ + 1; id < (int)robot.mBodies.size(); id++)
        actuatedJointIndices.push_back(id);

    // Initialize each constraint in the constraint set.
    // Also compute:
    //  - consDOFCount_: the number of constrained DOFs
    //  - unactDOFcount_: the number of unactuatuated DOFs
    std::set<int> slaveIds;
    for (auto const& constraint : constraintSet_)
    {
        // PRINT_DEBUG_STATEMENT("Initializing constraint \"" << constraint->getInstanceName() << "\", "
        //   << "enabled = " << constraint->isEnabled());

        constraint->init(robot);

        if (constraint->isEnabled())
        {
            consDOFcount_ += constraint->getNConstrainedDOFs();

            // If a constraint has a slave node it is a transmission constraint.
            // Save the ID of the slave node in slaveIds
            if (constraint->hasSlaveNode())
            {
                slaveIds.insert(constraint->getSlaveNode());

                // Remove the slave node from the list of actuated joints
                actuatedJointIndices.erase(find(actuatedJointIndices.begin(),
                    actuatedJointIndices.end(), constraint->getSlaveNode()));

                // Update the number of unactuated DOFs
                // NOTE: This assumes that each transmission constraint only restricts
                // one DOF.  May need to generalize this!
                unactDOFcount_ += 1;
            }
        }
    }

    // Resize internal storage.. this is just silly. Too many confusingly named variables
    this->resize(robot.dof_count, consDOFcount_, unactDOFcount_, virtualDOFcount_);

    // Set U
    // !!!WARNING!! No check for constraints in series!!!!!!
    int ii = 0; //row
    for(size_t jj = virtualDOFcount_; jj < robot.dof_count; jj++) //columns
    {
        auto id = slaveIds.find(jj+1); //RBDL index starts at 1, not 0
        if(id == slaveIds.end()) //node IS actuated
        {
            U_(ii++, jj) = 1.0;
            continue;
        }
    }

    // Set virtualU
    ii = 0; //row
    for(size_t jj = virtualDOFcount_; jj < robot.dof_count; jj++) //columns
    {
        virtualU_(ii++, jj) = 1.0;
    }

    if(getNConstraints() == 0)  // If there are no constraints, set UNc_ to be U_
        UNc_ = U_;

    initialized_ = true;

    // PRINT_DEBUG_STATEMENT("init complete. virtual DOF: " << virtualDOFcount_
    //   << ", number constrained DOF: " << consDOFcount_
    //   << ", number unactuated DOF: " << unactDOFcount_)

    // Update enableState_, a vector for remembering which constraints
    // were enabled when init(...) was last called.  It is used by enableStateChanged() to
    // detect if one or more of the constraints in the constraint set
    // was enabled or disabled since init(...) was called.
    enableState_.clear();
    for(auto const& constraint : constraintSet_)
    {
        enableState_.push_back(constraint->isEnabled());
    }

    return true;
}

bool ConstraintSet::enableSetChanged()
{
    for(size_t ii = 0; ii < constraintSet_.size(); ii++)
    {
        if (enableState_[ii] != constraintSet_[ii]->isEnabled())
        {
            PRINT_DEBUG_STATEMENT("Constraint " << constraintSet_[ii]->getInstanceName() << " changed enable state!")
            return true;
        }
    }
    return false;
}

void ConstraintSet::update(RigidBodyDynamics::Model& robot, const Vector& Q, const Matrix& Ainv)
{
    // Check if the enable/disable state of the constraints have changed.
    // If they have, re-initialize this constraint set.
    if (enableSetChanged())
        init(robot);

    if(getNConstraints() != 0) // non-empty constraint set, need to update Jc and Jc-derived quantities
    {
        updateJc(robot, Q);
        //update JcBar_
        Matrix temp1 = Jc_ * Ainv * Jc_.transpose();  // a version of Jc weighted by Ainv
        Matrix lambda1(temp1.cols(), temp1.rows());
        // pseudoInverse(temp1, sigmaThreshold_, lambda1, 0);
        // CONTROLIT_INFO << "Computing pseudoInverse";
        controlit::addons::eigen::pseudo_inverse(temp1, lambda1); //, sigmaThreshold_);
        JcBar_ = Ainv * Jc_.transpose() * lambda1;
        //update Nc_
        Nc_ = Id_col - JcBar_ * Jc_;
        //update UNc_
        UNc_ = U_ * Nc_;
    }

    //update UNcBar_
    UNcAiNorm_ = UNc_ * Ainv * UNc_.transpose(); // dimensions # actuable DOFs x # actuable DOFs
    Matrix lambda2(UNcAiNorm_.cols(), UNcAiNorm_.rows());
    // pseudoInverse(UNcAiNorm_, sigmaThreshold_, lambda2, 0);
    // CONTROLIT_INFO << "Computing pseudoInverse";
    controlit::addons::eigen::pseudo_inverse(UNcAiNorm_, lambda2); //, sigmaThreshold_);

    UNcBar_ = Ainv * UNc_.transpose() * lambda2;

  //   PRINT_DEBUG_STATEMENT(": Computed UNcBar:\n"
  //        " - UNcBar = \n" << UNcBar_ << "\n"
  //        " - Ainv = \n" << Ainv << "\n"
  //        " - UNc_.transpose() = \n" << UNc_.transpose() << "\n"
  //        " - lambda2 = \n" << lambda2 << "\n"
  //        " - UNcAiNorm_ = \n" << UNcAiNorm_ << "\n"
  //        " - UNc_ = \n" << UNc_)
}

ConstraintSet::ConstraintList_t const& ConstraintSet::getConstraintSet()
{
    assert(initialized_);
    return constraintSet_;
}

void ConstraintSet::addConstraint(Constraint* constraint)
{
    PRINT_DEBUG_STATEMENT("Method called!");
  
    // It is possible for a constraint to be added after this ConstraintSet is initialized.
    // When this happens, care must be taken to re-initialize this ConstraintSet.
    // assert(!initialized_);
  
    std::shared_ptr<Constraint> constraintSharedPointer(constraint);
    //Need to put a check here!!!!!
  
    // Tell our parent class this is another parameter collection
    if (!addParameterCollection(constraintSharedPointer))
    {
        CONTROLIT_PR_ERROR << "Failed to add constraint '" << constraint->getInstanceName() << "' as a parameter collection. Aborting add.";
        return;
    }
  
    constraintSet_.push_back(constraintSharedPointer);
  
    PRINT_DEBUG_STATEMENT("Added constraint '" << constraint->getInstanceName() << "'");
  
    // The constraint set just changed; declare it uninitialized.
    initialized_ = false;
}

//! Returns true if the node is constrained (not quite right)
// bool ConstraintSet::isConstrained(unsigned int id) const
// {
//   assert(initialized_);

//   for(auto const& constraint : constraintSet_)
//   {
//     if (constraint->isEnabled())
//     {
//       if (constraint->getMasterNode() == id)
//         return true;
//       else if (constraint->hasSlaveNode() && constraint->getSlaveNode() == id)
//         return true;
//     }
//   }
//   return false;
// }

ConstraintSet::ActuatedJointIds_t const& ConstraintSet::getActuatedJointIndices() const
{
    assert(initialized_);
    return actuatedJointIndices;
}

size_t ConstraintSet::getNConstraints() const
{
    size_t result = 0;
  
    for(auto const& constraint : constraintSet_)
    {
        if (constraint->isEnabled())
            result++;
    }
  
    return result;
}

void ConstraintSet::getU(Matrix& U) const
{
    // assert(initialized_);
    // if(U.rows() != U_.rows() || U_.cols() != U.cols())
    //     U.resize(U_.rows(), U_.cols());
    U = U_;
}

void ConstraintSet::getVirtualU(Matrix& virtualU) const
{
    // assert(initialized_);
    // if(virtualU.rows() != virtualU_.rows() || virtualU_.cols() != virtualU.cols())
    //     virtualU.resize(virtualU_.rows(), virtualU_.cols());
    virtualU = virtualU_;
}

void ConstraintSet::getJacobian(Matrix& Jc) const
{
    // assert(initialized_);
    // if(Jc.rows() != Jc_.rows() || Jc.cols() != Jc_.cols())
    //     Jc.resize(Jc_.rows(),Jc_.cols());
    Jc = Jc_;
}

void ConstraintSet::getJacobianBar(Matrix& JcBar) const
{
    // assert(initialized_);
    // if(JcBar.rows() != Jc_.cols() || JcBar.cols() != Jc_.rows())
    //     JcBar.resize(Jc_.cols(), Jc_.rows());
    JcBar = JcBar_;
}

void ConstraintSet::getNc(Matrix& Nc) const
{
    // assert(initialized_);
    // if(Nc.rows() != Jc_.cols() || Nc.cols() != Jc_.cols())
    //     Nc.resize(Jc_.cols(), Jc_.cols());
    Nc = Nc_;
}

void ConstraintSet::getUNc(Matrix& UNc) const
{
    // assert(initialized_);
    // if(UNc.rows() != U_.rows() || UNc.cols() != Jc_.cols())
    //     UNc.resize(U_.rows(), Jc_.cols());
    UNc = UNc_;
}

void ConstraintSet::getUNcAiNorm(Matrix& UNcAiNorm) const
{
    // assert(initialized_);
    // if(UNcAiNorm.rows() != UNc_.rows() || UNcAiNorm.cols() != UNc_.rows())
    //     UNcAiNorm.resize(UNc_.rows(), UNc_.rows());
    UNcAiNorm = UNcAiNorm_;
}

void ConstraintSet::getUNcBar(Matrix& UNcBar) const
{
    // assert(initialized_);
    // if(UNcBar.rows() != U_.cols() || UNcBar.cols() != U_.rows())
    //     UNcBar.resize(U_.cols(), U_.rows());
    UNcBar = UNcBar_;
}

unsigned int ConstraintSet::getNumUnactuatedDof() const
{
    // assert(initialized_);
    return unactDOFcount_;
}

// TODO: Rename this function .. makes no sense
unsigned int ConstraintSet::getNumVirtualDof() const
{
    // assert(initialized_);
    return virtualDOFcount_;
}

unsigned int ConstraintSet::getNConstrainedDOFs() const
{
    // assert(initialized_);
    return consDOFcount_;
}

void ConstraintSet::updateJc(RigidBodyDynamics::Model& robot, const Vector& Q)
{
    //assert(initialized_);
  
    int r = 0; //row counter
    for (auto & constraint : constraintSet_)
    {
        if (constraint->isEnabled())
        {
            //TODO: constraints store and initialize their own jacobian matrix!
            Matrix Jci(constraint->getNConstrainedDOFs(), robot.dof_count);
            constraint->getJacobian(robot, Q, Jci);
      
            for(size_t ii = 0; ii < constraint->getNConstrainedDOFs(); ii++)
                Jc_.row(r++) = Jci.row(ii);
        }
    }
}

void ConstraintSet::resize(unsigned int dof, unsigned int consDOFcount,
  unsigned int unactDOFcount, unsigned int virtualDOFcount)
{
    if (!constraintSet_.empty())
        Jc_.setZero(consDOFcount, dof);
    else
        Jc_.setIdentity(dof, dof);  // No constraints
  
    U_.setZero(dof - unactDOFcount - virtualDOFcount, dof);
    virtualU_.setZero(dof - virtualDOFcount, dof);
    JcBar_.setZero(Jc_.cols(), Jc_.rows());
    Nc_.setIdentity(Jc_.cols(), Jc_.cols());
    UNc_.setZero(U_.rows(), Nc_.cols());
    UNcAiNorm_.setZero(UNc_.rows(), UNc_.rows());
    UNcBar_.setZero(UNc_.cols(), UNc_.rows());
  
    Id_col.setIdentity(dof, dof);
    Id_row.setIdentity(consDOFcount, consDOFcount);
}

void ConstraintSet::dump(std::ostream& os, std::string const& prefix) const
{
    os << prefix << "ConstraintSet details:" << std::endl;
    os << prefix << "  Type: " << getTypeName() << std::endl;
    os << prefix << "  Name: " << getInstanceName() << std::endl;
    os << prefix << "  Constraints:" << std::endl;
  
    if (constraintSet_.size() == 0)
        os << prefix << "    [None]" << std::endl;
  
    for(auto & constraint : constraintSet_) // For each constraint
    {
        os << prefix << "    Constraint: "
          << "Type: " << constraint->getTypeName()
          << ", InstanceName: " << constraint->getInstanceName();
    
        if (constraint->isEnabled())
            os << ", Status: ENABLED" << std::endl;
        else
            os << ", Status: DISABLED" << std::endl;
    }
}

} // namespace controlit
