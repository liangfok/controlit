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

#include <controlit/VirtualLinkageModel.hpp>
#include <controlit/ConstraintSet.hpp>
#include <controlit/ContactConstraint.hpp>
#include <RigidBodyDynamics/Extras/rbdl_extras.hpp>
#include <controlit/addons/eigen/LinearAlgebra.hpp>
#include <controlit/addons/eigen/PseudoInverse.hpp>

#include <controlit/parser/yaml_parser.hpp>
#include <controlit/logging/RealTimeLogging.hpp>

namespace controlit {

using controlit::addons::eigen::pseudo_inverse;
using controlit::addons::eigen::Vector3d;
using controlit::addons::eigen::Matrix3d;

VirtualLinkageModel::VirtualLinkageModel() : 
    initialized_(false), exists_(true)
{}

VirtualLinkageModel::VirtualLinkageModel(std::string name) :
    initialized_(false), exists_(true)
{}

VirtualLinkageModel::~VirtualLinkageModel()
{
}

void VirtualLinkageModel::init(RigidBodyDynamics::Model& model, ConstraintSet& constraints)
{
    // Initialize the constraint set if necessary
    if(!constraints.isInitialized())
        constraints.init(model);
  
    // Save the enabled state of each constraint in the constraint set
    enableState_.clear();
    for(auto const& constraint : constraints.getConstraintSet())
    {
        enableState_.push_back(constraint->isEnabled());
    }
  
    // Save the sigma threshold
    sigmaThreshold_ = constraints.getSigmaThreshold();
  
    // Determine which constraints are due to contact and keep track of their rows
    size_t r = 0, ncum = 0; // number of active contraints
    size_t rcum = 0; // cumulative number of rows in the Jacobian for all active constraints
  
    contactConstraintIndices_.clear();
    worldCOPs_.clear();
    contactNormals_.clear();
    bodyId_.clear();
    contactSensorLocations_.clear();
    contactConstraintRowIndices_.clear();
    nrowsAndStartRow.clear();
  
    for(auto & constraint : constraints.getConstraintSet())
    {
        if (constraint->isEnabled())
        {
            if(constraint->isContactConstraint())
            {
                contactConstraintIndices_.push_back(ncum); // indices of the contact constraints in the constraint set
        
                // The COP, contact normal, and phi mapping might change during the motion
                worldCOPs_.emplace_back(3);
                contactNormals_.emplace_back(3);
                phiMaps_.push_back((static_cast<const ContactConstraint *>(constraint.get()))->getPhi());
        
                // These things should not change
                bodyId_.push_back(constraint->getMasterNode());
                contactSensorLocations_.push_back((static_cast<const ContactConstraint *>(constraint.get()))->getSensorLocation());
        
                size_t nrows = constraint->getNConstrainedDOFs();
                nrowsAndStartRow.push_back(std::make_pair(nrows, r)); //TODO--don't need both of these!!!
                size_t r0 = rcum;
                for(size_t jj = 0; jj < nrows; jj++)
                    contactConstraintRowIndices_.push_back(jj+r0);
         
                // increment counter for number of rows in Jc_
                r += nrows;
            }

            // Increment counters for number of constraints and number of rows in JcFull_
            rcum += constraint->getNConstrainedDOFs();
            ncum++;
        }
    }
  
    contactConstraintCount_ = contactConstraintIndices_.size();
    contactConstraintRows_ = contactConstraintRowIndices_.size();
  
    if(contactConstraintCount_ > 1)
    {
        for(size_t ii = 0; ii < contactConstraintCount_ - 1; ii++)
        {
            for(size_t jj = ii + 1; jj < contactConstraintCount_; jj++)
            {
                indexPairs_.push_back(std::make_pair(ii,jj));
                //deltaCOP_.emplace_back(3);
            }
        }
        pairCount_ = indexPairs_.size();
    
        resize(model.dof_count, constraints.getNConstrainedDOFs(), constraints.getNumUnactuatedDof(),
            constraints.getNumVirtualDof());
    
        r = 0; // reset row counter, prepare to set up selection matrices and Fr
        for(size_t ii = 0; ii < contactConstraintCount_ - 1; ii++)
        {
            selectionMatrixMn_(ii, ii * 3 + 2) = 1;
            selectionMatrixMt_(ii * 2, ii * 3) = 1;
            selectionMatrixMt_(ii * 2 + 1, ii * 3 + 1) = 1;
            for(size_t jj = ii + 1; jj < contactConstraintCount_; jj++)
            {
                selectionMatrixFt_(r, r * 3) = 1;
                r++;
            }
        }
    }

    // else if(contactConstraintCount_==1) //possible to still try to control moments....? (e.g. balance on one foot)
    // {
    //   pairCount_ = 0;
    //   resize(model.dof_count, constraints.getNConstrainedDOFs(), constraints.getNumUnactuatedDof(), constraints.getNumVirtualDof());
    // }
    else // No valid VLM model
    {
        exists_ = false;
        initialized_ = true;
        return;
    }

    size_t ii = contactConstraintCount_ - 1; // Fill in last columns of selection matrices (only columns if contactConstraintCount_ = 1)
    selectionMatrixMn_(ii, ii * 3 + 2) = 1;
    selectionMatrixMt_(ii * 2, ii * 3) = 1;
    selectionMatrixMt_(ii * 2 + 1, ii * 3 + 1) = 1;
  
    // Yes, this stores a copy of U_, but I'm not completely sure that transmission constraints are dealt with properly...so it's ok for now
    constraints.getU(U_);
  
    initialized_ = true;
}

bool VirtualLinkageModel::enableSetChanged(ConstraintSet& updatedConstraints)
{
    const ConstraintSet::ConstraintList_t & constraints = updatedConstraints.getConstraintSet();
  
    assert(enableState_.size() == constraints.size());
  
    for(size_t ii = 0; ii < constraints.size(); ii++)
    {
        if (enableState_[ii] != constraints[ii]->isEnabled())
        {
            CONTROLIT_INFO << "VirtualLinkageModel::enableSetChanged: Constraint '" << constraints[ii]->getInstanceName()
                           << "' changed enable state!";
            return true;
        }
    }
    return false;
}

void VirtualLinkageModel::update(RigidBodyDynamics::Model& robot, ConstraintSet& updatedConstraints,
    const Vector& Q, Matrix const& Ainv)
{
    // Ensure init() was called prior to calling this method.
    assert(initialized_);
  
    // Check if the set of enabled constraints have changed.
    // If they have changed, re-initialize this virtual linkage model.
    if (enableSetChanged(updatedConstraints))
        init(robot, updatedConstraints);
  
    if(!exists_)
        return;
  
    updateJc(updatedConstraints);
    updateWint(robot, updatedConstraints, Q); // Fr_ also updated here...
  
    if(updatedConstraints.getNConstraints() != contactConstraintCount_)
    {
        //update JcBar_
        Matrix temp1 = Jc_ * Ainv * Jc_.transpose();
        Matrix lambda1(temp1.cols(), temp1.rows());
        CONTROLIT_INFO << "Computing pseudoInverse";
        pseudo_inverse(temp1, lambda1); //, sigmaThreshold_);
        JcBar_ = Ainv * Jc_.transpose() * lambda1;
        //update Nc_
        Nc_ = Id_col - JcBar_ * Jc_;
        //update UNc_
        UNc_ = U_ * Nc_;
        //update UNcBar_
        Matrix temp2 = UNc_ * Ainv * UNc_.transpose();
        Matrix lambda2(temp2.cols(), temp2.rows());
        CONTROLIT_INFO << "Computing pseudoInverse";
        pseudo_inverse(temp2, lambda2); //, sigmaThreshold_);
        UNcBar_ = Ainv * UNc_.transpose() * lambda2;
    }
    else
    {
        // No need to repeat the calcuations if Jc_ = JcFull_
        updatedConstraints.getJacobianBar(JcBar_);
        updatedConstraints.getNc(Nc_);
        updatedConstraints.getUNc(UNc_);
        updatedConstraints.getUNcBar(UNcBar_);
    }
  
    //update Lstar_
    Lstar_ = Id_gamma - UNc_*UNcBar_;
    Matrix check = UNc_.transpose() * Lstar_;
}

void VirtualLinkageModel::getWint(Matrix& Wint) const
{
    // assert(initialized_);
    // if(Wint.rows()!=Wint_.rows()||Wint.cols()!=Wint_.cols())
        // Wint.resize(Wint_.rows(),Wint_.cols());
    Wint = Wint_;
}

void VirtualLinkageModel::getWintSensor(Matrix& WintSensor) const
{
    // assert(initialized_);
    // if(WintSensor.rows()!=WintSensor_.rows()||WintSensor.cols()!=WintSensor_.cols())
    //     WintSensor.resize(WintSensor_.rows(),WintSensor_.cols());
    WintSensor = WintSensor_;
}

void VirtualLinkageModel::getJacobian(Matrix& Jc) const
{
    // assert(initialized_);
    // if(Jc.rows()!=Jc_.rows()||Jc.cols()!=Jc_.cols())
    //     Jc.resize(Jc_.rows(),Jc_.cols());
    Jc = Jc_;
}

void VirtualLinkageModel::getJacobianBar(Matrix& JcBar) const
{
    // assert(initialized_);
    // if(JcBar.rows()!=Jc_.cols()||JcBar.cols()!=Jc_.rows())
    //     JcBar.resize(Jc_.cols(),Jc_.rows());
    JcBar = JcBar_;
}

void VirtualLinkageModel::getU(Matrix& U) const
{
    // assert(initialized_);
    // if(U.rows()!=U_.rows()||U.cols()!=U_.cols())
    //     U.resize(U_.rows(),U_.cols());
    U = U_;
}

void VirtualLinkageModel::getNc(Matrix& Nc) const
{
    // assert(initialized_);
    // if(Nc.rows()!=Jc_.cols()||Nc.cols()!=Jc_.cols())
    //     Nc.resize(Jc_.cols(),Jc_.cols());
    Nc = Nc_;
}

void VirtualLinkageModel::getUNc(Matrix& UNc) const
{
    // assert(initialized_);
    // if(UNc.rows()!=U_.rows()||UNc.cols()!=Jc_.cols())
    //     UNc.resize(U_.rows(),Jc_.cols());
    UNc = UNc_;
}

void VirtualLinkageModel::getUNcBar(Matrix& UNcBar) const
{
    // assert(initialized_);
    // if(UNcBar.rows()!=U_.cols()||UNcBar.cols()!=U_.rows())
    //     UNcBar.resize(U_.cols(),U_.rows());
    UNcBar = UNcBar_;
}

void VirtualLinkageModel::getLstar(Matrix& Lstar) const
{
    // assert(initialized_);
    // if(Lstar.rows()!=Lstar_.rows()||Lstar.cols()!=Lstar_.cols())
    //     Lstar.resize(Lstar_.rows(),Lstar_.cols());
    Lstar = Lstar_;
}

void VirtualLinkageModel::getFrSensor(Vector& FrSensor) const
{
    // assert(initialized_);
    // if(FrSensor.size()!=FrSensor_.size())
    //     FrSensor.resize(FrSensor_.size());
    FrSensor = FrSensor_;
}

bool VirtualLinkageModel::updateJc(ConstraintSet& constraints)
{
    assert(initialized_);
    constraints.getJacobian(JcFull_);
  
    // Convert the size of contactConstraintRowIndices_ (an unsigned value)
    // into a signed integer.
    int contactConstraintSize = 0;
    if (contactConstraintRowIndices_.size() < INT_MAX)
    {
        contactConstraintSize = (int)contactConstraintRowIndices_.size();
    }
    else
    {
        CONTROLIT_ERROR << "Cannot convert contactConstraintRowIndices_.size()"
                  << " into a signed integer because it exceeds the max integer value." << std::endl
                  << " - contactConstraintRowIndices_.size() = " << contactConstraintRowIndices_.size() << std::endl
                  << " - MAX_INT = " << INT_MAX;
        return false;
    }
  
    if(JcFull_.rows() == contactConstraintSize)
        Jc_ = JcFull_;
    else
    {
        size_t r = 0;
        for(auto & ii : contactConstraintRowIndices_)
            Jc_.row(r++) = JcFull_.row(ii);
    }
  
    return true;
}

void VirtualLinkageModel::updateWint(RigidBodyDynamics::Model& robot, ConstraintSet& constraints, const Vector& Q)
{

    assert(initialized_);
    size_t ii = 0;
    Matrix3d Rb; //Rotation from base to body coordinates.nm
    Matrix3d Rn; //Rotation from world normal direction to world x-axis
    //get updated COP location and reaction force information

    Vector3d worldSensorLocation;
    Matrix3d Ghat, Phat; //hat map of sensor location and COP, respectively.
    for(auto & index : contactConstraintIndices_)
    {
        //Need new COP and contact normal info
        worldCOPs_[ii] = (static_cast<const ContactConstraint *>(constraints.getConstraintSet()[index].get()))->getGoalWorldCOP();
        contactNormals_[ii] = (static_cast<const ContactConstraint *>(constraints.getConstraintSet()[index].get()))->getContactNormal();
        worldSensorLocation = CalcBodyToBaseCoordinates(robot,Q,bodyId_[ii],contactSensorLocations_[ii],false);
        FrSensor_.segment(ii*6,6) = (static_cast<const ContactConstraint *>(constraints.getConstraintSet()[index].get()))->getRxnForceCOM();
  
        Ghat = RigidBodyDynamics::Math::VectorCrossMatrix(worldSensorLocation);
        Phat = RigidBodyDynamics::Math::VectorCrossMatrix(worldCOPs_[ii]);
  
        Rb = RigidBodyDynamics::CalcBodyWorldOrientation(robot, Q, bodyId_[ii], false); //dynamics should be updated
        Rn = RigidBodyDynamics::Extras::calcRFromV1toV2(Rb.transpose() * contactNormals_[ii], Eigen::Vector3d::UnitZ());
  
        TsSensor_.block<3, 3>(ii * 3, ii * 6) = Rn * (Ghat - Phat);
        TsSensor_.block<3, 3>(ii * 3, ii * 6 + 3) = Rn;
        Ts_.block<3, 3>(ii * 3, ii * 6 + 3) = Rn;
  
        phiMaps_[ii] = (static_cast<ContactConstraint *>(constraints.getConstraintSet()[index].get()))->getPhi(Rn,Rb);
        Phi_.block(6 * ii, nrowsAndStartRow[ii].second, phiMaps_[ii].rows(), phiMaps_[ii].cols()) = phiMaps_[ii];
  
        ii++;
    }

    Vector3d deltaCOP;
    ii = 0; //reset counter to reuse it
    if(contactConstraintCount_ > 1)
    {
        Matrix3d Rt; // Rotation from direction uniting COPs to world x-axis
  
        for(auto & indexPair : indexPairs_) // loops through all pairs in indexPairs_
        {
            deltaCOP = worldCOPs_[indexPair.first]-worldCOPs_[indexPair.second];
    
            // Compute and fill in rotation matrices as block operation
    
            Rt = RigidBodyDynamics::Extras::calcRFromV1toV2(deltaCOP, Eigen::Vector3d::UnitX());
    
            if (!containerUtility.checkMagnitude(Rt, INFINITY_THRESHOLD))
            {
                CONTROLIT_ERROR_RT << "Invalid Rt!\n"
                  << std::scientific << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 1)
                  << " - Rt = " << Rt.transpose() << "\n"
                  << " - deltaCOP = " << deltaCOP.transpose() << "\n"
                  << " - ii = " << ii;
            }
    
            RtDeltaOperator_.block<3, 3>(ii * 3, indexPair.first * 6) = Rt;
            RtDeltaOperator_.block<3, 3>(ii * 3, indexPair.second * 6) = -Rt;
    
            // std::cout<<"Rt = \n"<<Rt<<std::endl;
            // std::cout<<"deltaCOP = "<<deltaCOP.transpose()<<std::endl;
    
            ii++;
        }

        // TODO: decrease FLOPs with explicit block operations!
        Wint_.topRows(pairCount_) = selectionMatrixFt_ * RtDeltaOperator_ * Phi_;
        Wint_.block(pairCount_,0,contactConstraintCount_ * 2, contactConstraintRows_) = selectionMatrixMt_ * Ts_ * Phi_;
        Wint_.bottomRows(contactConstraintCount_) = selectionMatrixMn_ * Ts_ * Phi_;

        // if (isnan(Wint_(0,0)))
        //   CONTROLIT_INFO_RT << "Wint_ contains NaN:\n" << Wint_;
        // if (!containerUtility.checkMagnitude(Wint_, INFINITY_THRESHOLD))
        // {
        //   CONTROLIT_ERROR_RT << "Invalid Wint!\n"
        //     << std::scientific << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 1)
        //     << " - Wint_ = " << Wint_ << "\n"
        //     << " - selectionMatrixFt_ =\n" << selectionMatrixFt_ << "\n"
        //     << " - RtDeltaOperator_ =\n" << RtDeltaOperator_ << "\n"
        //     << " - Phi_ =\n" << Phi_ << "\n"
        //     << " - selectionMatrixMt_ =\n" << selectionMatrixMt_ << "\n"
        //     << " - Ts_ =\n" << Ts_ << "\n"
        //     << " - Phi_ =\n" << Phi_ << "\n"
        //     << " - selectionMatrixMn_ = \n" << selectionMatrixMn_;
        // }
  
        WintSensor_.topRows(pairCount_) = selectionMatrixFt_ * RtDeltaOperator_;
        WintSensor_.block(pairCount_, 0, contactConstraintCount_ * 2, contactConstraintRows_) = selectionMatrixMt_ * TsSensor_;
        WintSensor_.bottomRows(contactConstraintCount_) = selectionMatrixMn_ * TsSensor_;
    }
    else
    {
        Wint_.block(pairCount_, 0, contactConstraintCount_ * 2, contactConstraintRows_) = selectionMatrixMt_ * Ts_ * Phi_;
        Wint_.bottomRows(contactConstraintCount_) = selectionMatrixMn_ * Ts_ * Phi_;
  
        WintSensor_.block(pairCount_, 0, contactConstraintCount_ * 2, contactConstraintRows_) = selectionMatrixMt_ * TsSensor_;
        WintSensor_.bottomRows(contactConstraintCount_) = selectionMatrixMn_ * TsSensor_;
    }

    // if (isnan(Wint_(0,0)))
    //     CONTROLIT_INFO_RT << "Wint_ contains NaN:\n" << Wint_;

}

void VirtualLinkageModel::resize(size_t dof, size_t consDOFcount, size_t unactDOFcount,
  size_t virtualDOFcount)
{
    // Things that might be returned
    Wint_.setZero(pairCount_ + contactConstraintCount_ * 3, contactConstraintRows_);
    WintSensor_.setZero(pairCount_ + contactConstraintCount_ * 3, contactConstraintCount_ * 6);
    Phi_.setZero(6 * contactConstraintCount_, contactConstraintRows_);
    U_.setZero(dof - unactDOFcount - virtualDOFcount, dof);
    Jc_.setZero(contactConstraintRows_, dof);
    JcFull_.setZero(consDOFcount, dof);
    JcBar_.setZero(Jc_.cols(), Jc_.rows());
    Nc_.setZero(Jc_.cols(), Jc_.cols());
    UNc_.setZero(U_.rows(), Nc_.cols());
    UNcBar_.setZero(UNc_.cols(), UNc_.rows());
    Lstar_.setZero(U_.rows(), U_.rows());
    FrSensor_.setZero(6 * contactConstraintCount_);
  
    // Things that probably need to be updated when Wint needs to be updated
    RtDeltaOperator_.setZero(3 * pairCount_, 6 * contactConstraintCount_);
    Ts_.setZero(3 * contactConstraintCount_, 6 * contactConstraintCount_);
    TsSensor_.setZero(3 * contactConstraintCount_, 6 * contactConstraintCount_);
  
    // Things that don't need to be updated when Wint is updated
    selectionMatrixFt_.setZero(pairCount_, 3 * pairCount_);//3*pairCount_);
    selectionMatrixMn_.setZero(contactConstraintCount_, 3 * contactConstraintCount_);//,3*contactConstraintCount_);
    selectionMatrixMt_.setZero(2 * contactConstraintCount_, 3 * contactConstraintCount_);//,2*contactConstraintCount_);
    Id_col.setIdentity(dof, dof);
    Id_row.setIdentity(contactConstraintRows_, contactConstraintRows_);
    Id_gamma.setIdentity(U_.rows(), U_.rows());
}

} // namespace controlit
