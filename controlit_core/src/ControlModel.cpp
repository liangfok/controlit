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

#include <assert.h>
#include <math.h> //cos, sin
#include <algorithm> //find

#include <controlit/ControlModel.hpp>
#include <controlit/ConstraintSetFactory.hpp>
#include <controlit_robot_models/rbdl_robot_urdfreader.hpp>
#include <controlit/addons/eigen/LinearAlgebra.hpp>

#include <iomanip>  // For std::setprecision and std::setw

#include <sstream> // For std::stringstream

#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Cholesky>

namespace controlit {

// Uncomment one of the following lines to enable/disable detailed debug statements.
#define PRINT_DEBUG_STATEMENT(ss)
// #define PRINT_DEBUG_STATEMENT(ss) CONTROLIT_DEBUG << ss;

#define PRINT_DEBUG_STATEMENT_RT(ss)
// #define PRINT_DEBUG_STATEMENT_RT(ss) CONTROLIT_DEBUG << ss;

ControlModel::ControlModel() :
    initialized_(false),
    isStale_(true),
    name("DEFAULT_MODEL"),
    params(nullptr)
{
}

ControlModel * ControlModel::createModel(ros::NodeHandle & nh,
    RobotState * latestRobotState, controlit::utility::ControlItParameters * params)
{
    PRINT_DEBUG_STATEMENT("Method called!");

    // Get the robot's URDF description from the ROS parameter server

    PRINT_DEBUG_STATEMENT("Getting the URDF description from the ROS parameter server.");

    std::string urdfDescription;
    if (!nh.getParam("controlit/robot_description", urdfDescription))
    {
        CONTROLIT_ERROR << "Parameter '" << nh.getNamespace() << "/controlit/robot_description' is not set!";
        return nullptr;
    }

    // Get the parameters from the ROS parameter server.  This contains the constraint
    // set definition.

    PRINT_DEBUG_STATEMENT("Getting the parameters from the ROS parameter server.");

    std::string yamlConfig;
    if (!nh.getParam("controlit/parameters", yamlConfig))
    {
        CONTROLIT_ERROR << "Parameter '" << nh.getNamespace() << "/controlit/parameters' is not set";
        return nullptr;
    }

    PRINT_DEBUG_STATEMENT("Creating the control model.");

    ControlModel * controlModel = createModel(urdfDescription, yamlConfig, latestRobotState, params);

    PRINT_DEBUG_STATEMENT("Done creating the control model.");

    return controlModel;
}

ControlModel* ControlModel::createModel(std::string const & urdfDescription,
    std::string const & yamlConfig, RobotState * latestRobotState, controlit::utility::ControlItParameters * params)
{
    PRINT_DEBUG_STATEMENT("Method called!");

    // Create and initialize the primary RBDL model
    RigidBodyDynamics::Model* model = new RigidBodyDynamics::Model();

    ControlModel::LinkNameToJointNameMap_t * l2jmap = new ControlModel::LinkNameToJointNameMap_t();

    bool verbose = false;

    PRINT_DEBUG_STATEMENT("Creating primary RBDL model....");
    if (!rbdl_robot_urdfreader::read_urdf_model_from_string(urdfDescription, model, l2jmap, nullptr, verbose))
    {
        CONTROLIT_ERROR << "Unable to read URDF File and create the primary RBDL robot model!";
        delete model;
        return nullptr;
    }

    PRINT_DEBUG_STATEMENT("Saving the joint limits into the ControlItParameters object.");

    // Create the constraint set
    PRINT_DEBUG_STATEMENT("Creating the constraint set...");
    controlit::ConstraintSetFactory constraintSetFactory;

    controlit::ConstraintSet* constraintSet = nullptr;
    if (!yamlConfig.empty()) constraintSet = constraintSetFactory.loadFromString(yamlConfig);

    if (constraintSet == nullptr)
    {
        // Assume no constraints
        constraintSet = new ConstraintSet();
        CONTROLIT_WARN << "No constraints found in YAML spec. Assuming no constraints.";
    }

    // Create the control model
    ControlModel* controlModel = new ControlModel();
    if (!controlModel->init(model, latestRobotState, l2jmap, constraintSet, params))
    {
        CONTROLIT_ERROR << "Failed initialization of control model, returning null.";
        return nullptr;
    }

    // Initialize model for SysId.
    // controlModel->initSysId(urdfDescription);

    PRINT_DEBUG_STATEMENT("Done creating control model!");
    return controlModel;
}

// void ControlModel::initSysId(const std::string &urdfDescription) {
//     //! @todo get the joint fix list, probably should be a static variable somewhere.
//     //! @todo How to construct model from urdf, from where to get the urdf?
//     rbdlSysIdModel_.reset(new RigidBodyDynamics::Model());
//     rbdlSysIdModel_->Init();

//     sysid.reset(new nasa::sysid::SysId(urdfDescription, rbdlSysIdModel_.get()));
//     sysid->init();
//     sysid->start();
// }

bool ControlModel::init(RigidBodyDynamics::Model * primaryModel,
    RobotState * latestRobotState,
    ControlModel::LinkNameToJointNameMap_t * linkNameToJointNameMap,
    ConstraintSet * constraints,
    controlit::utility::ControlItParameters * params)
{
    // Ensure this ControlModel is only be initialized once.
    if(initialized_)
    {
        CONTROLIT_WARN << "Attempted to initialize more than once!";
        return true;
    }

    // Save the parameters.
    rbdlModel_.reset(primaryModel);
    this->latestRobotState = latestRobotState;
    linkNameToJointNameMap_.reset(linkNameToJointNameMap);
    constraints_.reset(constraints);
    virtualLinkageModel_.reset(new VirtualLinkageModel());

    PRINT_DEBUG_STATEMENT("Saving pointer to ControlIt! parameters. params = " << params);
    this->params = params;

    if (params->hasReflectedRotorInertias())
    {
        const Vector & rri = params->getReflectedRotorInertias();
        if ((unsigned int)rri.size() == primaryModel->dof_count)
        {
            for (int ii = 0; ii < rri.size(); ii++)
            {
                reflectedRotorInertias.push_back(rri[ii]);
            }  
        }
        else
        {
            CONTROLIT_ERROR << "Reflected rotor inertias has incorrect length!\n"
                               "  - expected: " << primaryModel->dof_count << "\n"
                               "  - got: " << reflectedRotorInertias.size();
            return false;
        }
    } 
    else
    {
        for (size_t ii = 0; ii < primaryModel->dof_count; ii++)
        {
          reflectedRotorInertias.push_back(0);
        }
    }

    return reinit();
}

bool ControlModel::reinit()
{
    // Initialize the constraint set.  This initializes U and resizes Jc and
    // other derived quantities as needed
    if(!constraints_->init(*(rbdlModel_.get())))
    {
        CONTROLIT_ERROR << "Failed to initialize constraint set!!";
        return false;
    }

    // Set up virtual linkage model based on constraint set and RBDL model
    virtualLinkageModel_->init(*(rbdlModel_.get()), *(constraints_.get()));

    // Save the number of DOFs (actual and virtual)
    numDOFs = rbdlModel_->dof_count;

    // Calculate and save the number of actuatable DOFs
    NActuableDOFs_ = numDOFs - constraints_->getNumVirtualDof() - constraints_->getNumUnactuatedDof();

    // Calculate and save the number of real DOFs
    NRealDOFs_ = numDOFs - constraints_->getNumVirtualDof();

    // Resize and zero-out internal matrices
    Q_.setZero(numDOFs);
    Qd_.setZero(numDOFs);
    Qdd_.setZero(numDOFs);

    A_.setZero(numDOFs, numDOFs);
    UnmolestedA_.setZero(numDOFs, numDOFs);
    Ainv_.setZero(numDOFs, numDOFs);
    UnmolestedAinv_.setZero(numDOFs, numDOFs);
    AMask_.setOnes(numDOFs, numDOFs);
    grav_.setZero(numDOFs);
    gravMask_.setOnes(numDOFs);

    // Initialize the joint state time stamp to be now
    jointStateTimeStamp = high_resolution_clock::now();

    // Update the vector that contains the actuated joint names
    actuatedJointNames.clear();
    const ConstraintSet::ActuatedJointIds_t & actuatedJointIndices = constraints_->getActuatedJointIndices();
    for (size_t ii = 0; ii < actuatedJointIndices.size(); ii++)
    {
        actuatedJointNames.push_back(rbdlModel_->GetBodyName(actuatedJointIndices[ii]));
    }

    // Set the vector of real joint names
    realJointNames.clear();
    for(size_t ii = constraints_->getNumVirtualDof() + 1; ii <= (size_t) numDOFs; ii++)
    {
        realJointNames.push_back(rbdlModel_->GetBodyName(ii));
    }

    initialized_ = true;

    // For visualizing joint state information in rViz
    // setupJointStatePubs();

    return true;
}

bool ControlModel::setAMask(const std::vector<std::vector< std::string>> & jointGroups)
{
    if(!initialized_)
    {
        CONTROLIT_ERROR << "Cannot call setAMask unless ControlModel has been initialized!!";
        return false;
    }
  
    // Reset the AMask to be all ones (all joints coupled)
    AMask_.setOnes(numDOFs, numDOFs);
    std::vector<std::vector<unsigned int>> jointGroupIndices;
  
    // Get the indices of all of the joints
    for(size_t i = 0; i < jointGroups.size(); i++)
    {
        std::vector<unsigned int> group(jointGroups[i].size(),0);
        for(size_t j = 0; j < jointGroups[i].size(); j++)
        {
            group[j] = rbdlModel_->GetBodyId(jointGroups[i][j].c_str()) - 1;
        }
        jointGroupIndices.push_back(group);
    }
  
    //Set up the AMask_ matrix
    //Look through each coupled group
    for(size_t i = 0; i < jointGroupIndices.size(); i++)
    {
        std::vector<unsigned int>::iterator endId = jointGroupIndices[i].end();
        for(int k = 6; k < AMask_.rows(); k++)
        {
            //look for index k in the current group
            std::vector<unsigned int>::iterator it;
            it = std::find(jointGroupIndices[i].begin(), jointGroupIndices[i].end(), k);
            if(it == endId) // joint k is NOT in this group!
            {
                for(size_t j = 0; j < jointGroupIndices[i].size(); j++)
                {
                    // Convert AMask.rows(), a signed integer, into an unsigned integer.
                    size_t aMaskRows = (size_t)AMask_.rows();
          
                    if(jointGroupIndices[i][j] < aMaskRows) // just in case std::numeric_limits
                    {
                        AMask_(k, jointGroupIndices[i][j]) = 0.0;
                        AMask_(jointGroupIndices[i][j], k) = 0.0;
                    }
                }
            }
        }
    }
  
    return true;
}

bool ControlModel::setGravMask(const std::vector<std::string> & maskedJoints)
{
    if(!initialized_)
    {
        CONTROLIT_ERROR << "Cannot call setGravMask unless ControlModel has been initialized!!";
        return false;
    }
    // Reset the gravity mask to be all ones (no joints masked)
    gravMask_.setOnes(numDOFs);
  
    std::vector< size_t > maskedJointIndices;
  
    // Get the indices of the masked joints
    for(size_t i = 0; i < maskedJoints.size(); i++)
    {
        maskedJointIndices.push_back(rbdlModel_->GetBodyId(maskedJoints[i].c_str()) - 1);
    }
  
    // Setup the gravMask_ vector
    for(size_t i = 0; i < maskedJointIndices.size(); i++)
    {
        controlit_assert_msg(gravMask_.size() >= 0, "Unable to convert gravMask.size() to be an unsigned integer because it is negative.");
        size_t gravMaskSize = (size_t)gravMask_.size();
    
        if(maskedJointIndices[i] < gravMaskSize)
        {
            gravMask_[maskedJointIndices[i]] = 0.0;
        }
        else
        {
            CONTROLIT_ERROR 
                << "Masked joint index of " << maskedJointIndices[i]
                << " exceeds size of gravity vector (" << gravMaskSize << ")";
        }
    }
  
    return true;
}

RigidBodyDynamics::Model& ControlModel::rbdlModel()
{
    assert(initialized_);
    return *(rbdlModel_.get());
}

RigidBodyDynamics::Model const& ControlModel::rbdlModel() const
{
    assert(initialized_);
    return *(rbdlModel_.get());
}

ControlModel::LinkNameToJointNameMap_t& ControlModel::linkNameToJointNameMap()
{
    assert(initialized_);
  
    // Verify that linkNameToJointNameMap_ is set.
    if (linkNameToJointNameMap_ == NULL)
    {
        CONTROLIT_ERROR << "linkNameToJointNameMap_ not set!";
        assert(false);
    }
  
    return *(linkNameToJointNameMap_.get());
}

ControlModel::LinkNameToJointNameMap_t const& ControlModel::linkNameToJointNameMap() const
{
    assert(initialized_);
  
    // Verify that linkNameToJointNameMap_ is set.
    if (linkNameToJointNameMap_ == NULL)
    {
        CONTROLIT_ERROR << "linkNameToJointNameMap_ not set!";
        assert(false);
    }
  
    return *(linkNameToJointNameMap_.get());
}

//! Get a reference to a whole-body external constraint description
ConstraintSet& ControlModel::constraints()
{
    assert(initialized_);
    return *(constraints_.get());
}

ConstraintSet const& ControlModel::constraints() const
{
    assert(initialized_);
    return *(constraints_.get());
}

//! Get a reference to a virtual linkage model
VirtualLinkageModel& ControlModel::virtualLinkageModel()
{
    assert(initialized_);
    return *(virtualLinkageModel_.get());
}

VirtualLinkageModel const& ControlModel::virtualLinkageModel() const
{
    assert(initialized_);
    return *(virtualLinkageModel_.get());
}

void ControlModel::update()
{
    PRINT_DEBUG_STATEMENT("Method called!\n"
      " - name = " << name << "\n"
      " - # actuable DOFs: " << getNActuableDOFs() << "\n"
      " - # total DOFs: " << getNumDOFs() << "\n"
      " - Q: " << Q_.transpose() << "\n"
      " - Qd: " << Qd_.transpose()) // << "\n"
      // " - gravMask: " << gravMask_.transpose() << "\n"
      // " - AMask:\n" << AMask_;
  
    assert(initialized_);
  
    /*
     * Selectively update the kinematics of the robot model.
     * In this case, only update the joint positions, not the
     * joint velocities or accelerations.
     *
     * QUESTION 1: Is it necessary to set the joint velocities
     * and accelerations to zero to properly compute gravity
     * and the inertia matrix?
     *
     * QUESTION 2: Why don't we remove this method call and instead
     * change the last parameter of the next method call to be true?
     *
     * See: http://rbdl.bitbucket.org/de/d92/group__kinematics__group.html#gab748e2c620c3129e94e0c6665cd6638d
     */
     RigidBodyDynamics::UpdateKinematicsCustom(*(rbdlModel_.get()), &Q_, NULL, NULL);
     // RigidBodyDynamics::UpdateKinematics(*(rbdlModel_.get()), Q_, Qd_, Qdd_);
  
    /*
     * Compute the joint space inertia matrix by using the Composite Rigid Body Algorithm.
     * In this case, the matrix 'A' is the joint state inertia matrix, which is
     * where the results are stored.
     *
     * The last parameter specifies whether to update the kinematics.  It is false
     * since the kinematics were just updated.
     *
     * See: http://rbdl.bitbucket.org/d6/d63/group__dynamics__group.html#ga673f38a3cb6fce883ec5d52ad6f3f6e0
     */
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(*(rbdlModel_.get()), Q_, A_, false);
  
    // Account for rotor inertias
    // CONTROLIT_INFO_RT << "Inertia matrix prior to adding rotor inertia:\n" << A_;
    for (size_t ii = 0; ii < reflectedRotorInertias.size(); ii++)
    {
      A_(ii, ii) += reflectedRotorInertias[ii];
    }
    // CONTROLIT_INFO_RT << "Inertia matrix after adding rotor inertia:\n" << A_;
  
    /*!
     * Apply a mask on the A matrix.  This is used to decouple certain links in the
     * robot from other links.
     */
    UnmolestedA_ = A_;
    A_ = A_.cwiseProduct(AMask_);
  
  #ifdef WBC_EXTRA_CONSISTENCY_CHECKS
    /*!
     * Define the threshold above which a value is considered to be infinity.
     */
    static const double INFINITY_THRESHOLD = 1e10;
  
    // Ensure 'A' matrix is valid
    controlit_assert_msg(controlit::addons::eigen::checkRange(A_, -INFINITY_THRESHOLD, INFINITY_THRESHOLD), "Invalid 'A' matrix after update.");
  #endif
  
    // Compute gravity
    // RigidBodyDynamics::InverseDynamics(*(rbdlModel_.get()), Q_, Vector::Zero(Q_.size()), Vector::Zero(Q_.size()), grav_);
    RigidBodyDynamics::InverseDynamics(*(rbdlModel_.get()), Q_, Qd_, Vector::Zero(Q_.size()), grav_);
  
    /*!
     * Apply a mask on the gravity vector.  This is used to remove
     * the force of gravity from certain links on the robot.
     */
    grav_ = gravMask_.cwiseProduct(grav_);
  
    // PRINT_DEBUG_STATEMENT("After applying gravity mask:\n"
    //   " - grav_ = " << grav_.transpose());
  
    /*!
     * Update Ainv_, the inverse of A.  The inverse of A is useful for
     * computing how the robot will accerate given the application
     * of a particular torque or force.
     */
     Ainv_ = A_.inverse(); // TODO: be smarter and do this faster!
  
    /*
     * Update the kinematics of the robot model.
     * In this case, update the joint positions, velocities, and accelerations.
     *
     * See: http://rbdl.bitbucket.org/de/d92/group__kinematics__group.html#gab748e2c620c3129e94e0c6665cd6638d
     */
    RigidBodyDynamics::UpdateKinematicsCustom(*(rbdlModel_.get()), &Q_, &Qd_, &Qdd_);
    // std::cout<<"first 6 dofs = "<<Q_.head(6).transpose()<<std::endl;
  
    /*!
     * Update the ConstraintSet.  This includes updating the constraint Jacobian matrix.
     */
    constraints_->update(*(rbdlModel_.get()), Q_, Ainv_);
  
    /*!
     * Update the VirtualLinkageModel.
     */
    virtualLinkageModel_->update(*(rbdlModel_.get()), *(constraints_.get()), Q_, Ainv_);
  
    isStale_ = false;
}

void ControlModel::updateJointState()
{
    // Verify that the sizes of q, qd, and qdd equal the number of real joints
    // in the model.
    controlit_assert_msg((int)latestRobotState->getNumJoints() == getNumRealDOFs(),
        "The number of real joints in latestRobotState " << latestRobotState->getNumJoints() << 
        " does not match the number of real joints in the model " << getNumRealDOFs());
  
    // Save the joint positions, velocities, and accelerations in local member variables.
    Q_.block(6, 0, getNumRealDOFs(), 1) = latestRobotState->getJointPosition();
    Qd_.block(6, 0, getNumRealDOFs(), 1) = latestRobotState->getJointVelocity();
    Qdd_.block(6, 0, getNumRealDOFs(), 1) = latestRobotState->getJointAcceleration();
  
    Q_.segment(0, 6) = latestRobotState->getVirtualJointPosition();
    Qd_.segment(0,6) = latestRobotState->getVirtualJointVelocity();
  
    // Save the timestamp of the joint state information.
    // This is useful for computing how "stale" the ControlModel is
    // when the servo loop's RT thread computes the next command.
    jointStateTimeStamp = latestRobotState->getTimestamp();
}

void ControlModel::setFullJointState(Vector const& q, Vector const& qd, Vector const& qdd)
{
    Q_ = q;
    Qd_ = qd;
    Qdd_ = qdd;
}

void ControlModel::getLatestFullState(Vector & Q, Vector & Qd)
{
    // PRINT_DEBUG_STATEMENT("Method called\n"
    //   << " - Num DOFs: " << getNumDOFs() << "\n"
    //   << " - Num Virtual DOFs: " << getNumVirtualDOFs());
  
    // Ensure the size of the storage vectors are correct
    if (Q.size() != getNumDOFs())
        Q.resize(getNumDOFs());
  
    if (Qd.size() != getNumDOFs())
        Qd.resize(getNumDOFs());
  
    int numVirtualDOFs = getNumVirtualDOFs();
  
    // Save the latest virtual DOF state
    Q.segment(0, numVirtualDOFs) = latestRobotState->getVirtualJointPosition();
    Qd.segment(0, numVirtualDOFs) = latestRobotState->getVirtualJointVelocity();
  
    // Save the latest actual DOF state
    Q.segment(numVirtualDOFs, numDOFs - numVirtualDOFs) = latestRobotState->getJointPosition();
    Qd.segment(numVirtualDOFs, numDOFs - numVirtualDOFs) = latestRobotState->getJointVelocity();
}

bool ControlModel::getFrameID(std::string & name, int & bodyID) const
{
    PRINT_DEBUG_STATEMENT("Method called, name '" << name << "'.");

    // Verify that this ControlModel is initialized.
    if (!initialized_)
    {
        CONTROLIT_ERROR << "ControlModel not initialized!";
        return false;
    }

    // Verify that parameter 'name' is valid
    if (name.empty())
    {
        CONTROLIT_ERROR << "Body name not specified!";
        return false;
    }

    // If parameter 'name' is "world", set the bodyID equal to -1.
    if (name.compare("world") == 0)
    {
        // CONTROLIT_DEBUG << "Frame is World! Returning bodyID = -1.";
        bodyID = -1;
        return true;
    }
  
    bool foundBody = true;
  
    // Check if parameter "name" is of a joint.
    unsigned int unSignedBodyID = rbdlModel_->GetBodyId(name.c_str());
  
    if (unSignedBodyID == controlit::addons::eigen::RBDL_BODY_NOT_FOUND)
    {
        // Parameter "name" is not of a joint, see if it is of a body.
        LinkNameToJointNameMap_t::iterator it = linkNameToJointNameMap_->find(name);
    
        if(it != linkNameToJointNameMap_->end())
        {
            std::string jointName = it->second.c_str();
            unSignedBodyID = rbdlModel_->GetBodyId(jointName.c_str());
      
            if (unSignedBodyID == controlit::addons::eigen::RBDL_BODY_NOT_FOUND)
                foundBody = false; // parameter "name" mapped to a joint that is not in the model!
        }
        else
          foundBody = false; // parameter "name" is not of a body
    
        // Report an error if the body was not found.
        if (!foundBody)
        {
            // Get a list of links and joints
            std::string linkNames;
            std::string jointNames;
            if (!getLinkAndJointNames(linkNames, jointNames))
            {
                linkNames = "[NONE]";
                jointNames = "[NONE]";
            }
      
            CONTROLIT_ERROR 
                << "Could not find body or joint named \"" << name << "\"!\n"
                << " - links: " << linkNames << "\n"
                << " - joints: " << jointNames;
            return false;
        }
    }
  
    // Convert the unsigned bodyID into a signed value
    controlit_assert_msg(unSignedBodyID <= (unsigned int)std::numeric_limits<int>::max(),
        "Unable to convert unsigned bodyID of " << unSignedBodyID << " into a signed value.");
    bodyID = (int)unSignedBodyID;
  
    return true;
}

bool ControlModel::getBodyID(const std::string & name, unsigned int & bodyID) const
{
    // Verify that this ControlModel is initialized.
    if (!initialized_)
    {
        CONTROLIT_ERROR << "ControlModel not initialized!";
        return false;
    }
  
    if (name.empty())
    {
        CONTROLIT_ERROR << "Name not specified!";
        return false;
    }
  
    bool foundBody = true;
  
    // Check if parameter "name" is that of a joint.
    bodyID = rbdlModel_->GetBodyId(name.c_str());
  
    if (bodyID == controlit::addons::eigen::RBDL_BODY_NOT_FOUND)
    {
        // Parameter "name" is not of a joint, see if it is of a body.
        LinkNameToJointNameMap_t::iterator it
            = linkNameToJointNameMap_->find(name);
    
        if(it != linkNameToJointNameMap_->end())
        {
            std::string jointName = it->second.c_str();
            bodyID = rbdlModel_->GetBodyId(jointName.c_str());
      
            if (bodyID == controlit::addons::eigen::RBDL_BODY_NOT_FOUND)
                foundBody = false; // parameter "name" mapped to a joint that is not in the model!
        }
        else
            foundBody = false; // parameter "name" is not of a body
    
        // Report an error if the body was not found.
        if (!foundBody)
        {
            // Get a list of links and joints
            std::string linkNames;
            std::string jointNames;
            if (!getLinkAndJointNames(linkNames, jointNames))
            {
                linkNames = "[NONE]";
                jointNames = "[NONE]";
            }
      
            CONTROLIT_ERROR << "Could not find body or joint named \"" << name << "\"!\n"
                            << " - links: " << linkNames << "\n"
                            << " - joints: " << jointNames;
            return false;
        }
    }
    return true;
}

bool ControlModel::getJointIndex(const std::string & name, unsigned int & index) const
{
    unsigned int bodyID;
  
    // Abort if we failed to get the bodyID
    if (!getBodyID(name, bodyID)) return false;
  
    index = bodyID - getNumVirtualDOFs() - 1; // subtract 1 to account for RBDL's "root" node
    return true;
}

bool ControlModel::getLinkCOMs(Matrix & linkCOMs)
{
    const ConstraintSet::ActuatedJointIds_t & actuatedJointIndices = constraints_->getActuatedJointIndices();
  
    // Convert linkCOMs.cols() into an unsigned integer
    controlit_assert_msg(linkCOMs.cols() >= 0,
        "Unable to convert linkCOMs.cols() into an unsigned integer because it is negative.");
    size_t linkCOMCols = (size_t)linkCOMs.cols();
  
    // If necessary resize matrix 'linkCOMs'.
    if (linkCOMs.rows() != 3 || linkCOMCols != actuatedJointIndices.size())
        linkCOMs.resize(3, actuatedJointIndices.size());
  
    // Zero matrix 'linkCOMs'
    linkCOMs.setZero(3, actuatedJointIndices.size());
  
    int linkCOMsIndx = 0;   // An index into where to save the COM into matrix linkCOMs
  
    // For each actuated joint, computes its COM in the world coordinate frame
    // and save it in the 'linkCOMs' matrix.
    for (size_t ii = 0; ii < actuatedJointIndices.size(); ii++)
    {
        int currJointIndx = actuatedJointIndices[ii];
        Vector COMi = RigidBodyDynamics::CalcBodyToBaseCoordinates(
            rbdlModel(),
            Q_,
            currJointIndx,
            rbdlModel_->mBodies[currJointIndx].mCenterOfMass,
            false);
    
        linkCOMs.block(0, linkCOMsIndx, COMi.size(), 1) = COMi;  // Save the results
        linkCOMsIndx++;
    }
  
    return true;
}

void ControlModel::printState()
{
    // Get a reference to the robot model
    RigidBodyDynamics::Model & model = this->rbdlModel();
  
    // Get the mapping between body name to body ID
    std::map<std::string, unsigned int> bodyNameToIDMap = model.mBodyNameMap;
  
    // Get an iterator to the above map.
    std::map<std::string, unsigned int>::iterator iter;
  
    // Create string buffer for holding the output string
    std::stringstream msgBuff;
  
    // msgBuff << std::scientific << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 1);
  
    // Save a table header into the string buffer
    // msgBuff << std::setw(25) << "Body Name" << std::setw(10) << "Body ID" << std::setw(20) << "World Position" << std::setw(20) << "World Velocity" <<"\n";
  
    // For each body in the model, add a row to the string buffer with its details
    for (iter = bodyNameToIDMap.begin(); iter != bodyNameToIDMap.end(); ++iter)
    {
        std::string bodyName = iter->first;
        unsigned int bodyID = iter->second;
    
        bool updateKinematics = false;
        Vector controlPoint(3); controlPoint.setZero(); // the body-frame coordinate for which to compute the global coordinate
    
        Vector bodyPos = RigidBodyDynamics::CalcBodyToBaseCoordinates(model, this->getQ(), bodyID, controlPoint, updateKinematics);
        Vector bodyVel = RigidBodyDynamics::CalcPointVelocity(model, this->getQ(), this->getQd(), bodyID, controlPoint, updateKinematics);
    
        std::stringstream bodyPosBuff;
        bodyPosBuff << bodyPos.transpose();
    
        std::stringstream bodyVelBuff;
        bodyVelBuff << bodyVel.transpose();
    
        // msgBuff << std::setw(25) << bodyName << std::setw(10) << bodyID << std::setw(20) << bodyPosBuff.str() << std::setw(20) << bodyVelBuff.str() << "\n";
        msgBuff << "  - bodyName = " << bodyName << ", bodyID = " << bodyID << ", position = " << bodyPos.transpose() << ", bodyVel = " << bodyVel.transpose() << "\n";
    
        // // Get the joint state publisher
        // ros::Publisher & publisher = jointStatePublishers[bodyName];
    
        // // Save the COM in a message
        // geometry_msgs::PoseStamped jointStateMsg;
    
        // jointStateMsg.header.frame_id = "/world";
        // jointStateMsg.header.stamp = ros::Time::now();
        // jointStateMsg.header.seq = 0;
        // jointStateMsg.pose.position.x = bodyPos[0];
        // jointStateMsg.pose.position.y = bodyPos[1];
        // jointStateMsg.pose.position.z = bodyPos[2];
        // jointStateMsg.pose.orientation.x = 0.0;
        // jointStateMsg.pose.orientation.y = 0.0;
        // jointStateMsg.pose.orientation.z = 0.0;
        // jointStateMsg.pose.orientation.w = 0.0;
    
        // // Publish the joint state
        // publisher.publish(jointStateMsg);
  
    }
  
    // Print the resulting table
    CONTROLIT_INFO << "Current State of model: " << std::endl << msgBuff.str();
}

void ControlModel::dumpLinkNameToJointNameMap(std::ostream& os) const
{
    os << "Contents of linkNameToJointNameMap:\n";
  
    if (linkNameToJointNameMap_ == NULL)
    {
        CONTROLIT_WARN << "linkNameToJointNameMap_ not set!";
        os << "[Map is NULL]";
        return;
    }
  
    for(std::map<std::string, std::string>::const_iterator it = linkNameToJointNameMap_->begin();
        it != linkNameToJointNameMap_->end(); it++)
    {
        os << "Key: " << it->first << ", Value: " << it->second << "\n";
    }
}

std::string ControlModel::getActuatedJointNames() const
{
    std::stringstream jointNameBuff;
  
    const ConstraintSet::ActuatedJointIds_t & actuatedJointIndices = constraints_->getActuatedJointIndices();
    for (size_t ii = 0; ii < actuatedJointIndices.size(); ii++)
    {
        jointNameBuff << rbdlModel_->GetBodyName(actuatedJointIndices[ii]) << ", ";
    }
    std::string jointNames = jointNameBuff.str();
    jointNames = jointNames.substr(0, jointNames.size() - 2);
  
    return jointNames;
}

const std::vector<std::string> & ControlModel::getRealJointNamesVector() const
{
    return realJointNames;
}

const std::vector<std::string> & ControlModel::getActuatedJointNamesVector() const
{
    return actuatedJointNames;
}

const std::string ControlModel::getBaseLinkName() const
{
    PRINT_DEBUG_STATEMENT("Method called! params = " << params);
    if (params->hasModelBaseLinkName())
    {
        return params->getModelBaseLinkName();
    }
    else
    {
        // Search linkNameToJointNameMap for the link corresponding to the first joint.
        // The assumption here is that the first joint is called "rigid6DoF".
        for(std::map<std::string, std::string>::const_iterator it = linkNameToJointNameMap_->begin();
            it != linkNameToJointNameMap_->end(); it++)
        {
            if (it->second.compare("rigid6DoF") == 0)
                return it->first;
        }

        CONTROLIT_WARN << "Unable to find base link name!";
        assert(false);

        return "UNKNOWN_BASE_LINK_NAME";
    }
}

bool ControlModel::getLinkAndJointNames(std::string & linkNames, std::string & jointNames) const
{
    std::stringstream linkNameBuff;
    std::stringstream jointNameBuff;
  
    if (linkNameToJointNameMap_ == NULL)
    {
        CONTROLIT_ERROR << "linkNameToJointNameMap_ not set!";
        return false;
    }
  
    for(std::map<std::string, std::string>::const_iterator it = linkNameToJointNameMap_->begin();
        it != linkNameToJointNameMap_->end(); it++)
    {
        linkNameBuff << it->first << ", ";
        jointNameBuff << it->second << ", ";
    }
  
    // save the results
    linkNames = linkNameBuff.str();
    jointNames = jointNameBuff.str();
  
    // Remove trailing comma
    linkNames = linkNames.substr(0, linkNames.size() - 2);
    jointNames = jointNames.substr(0, jointNames.size() - 2);
  
    return true;
}

double ControlModel::getAge()
{
    high_resolution_clock::time_point currTime = high_resolution_clock::now();
    std::chrono::nanoseconds timeSpan = duration_cast<std::chrono::nanoseconds>(
        currTime - jointStateTimeStamp);
    return timeSpan.count() / 1e9;
}

ros::Time & ControlModel::getTimeStamp()
{
    std::chrono::nanoseconds timeSinceEpoch_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        jointStateTimeStamp.time_since_epoch());
    
    double timeSinceEpoch_s = timeSinceEpoch_ns.count() / 1e9;
  
    jointStateROSTimeStamp.sec = (long)timeSinceEpoch_s;
    jointStateROSTimeStamp.nsec = (long)((timeSinceEpoch_s - jointStateROSTimeStamp.sec) * 1e9);
  
    return jointStateROSTimeStamp;
}

} // namespace controlit
