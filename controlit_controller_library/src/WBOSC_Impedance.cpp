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

#include <controlit/controller_library/WBOSC_Impedance.hpp>
#include <controlit/addons/eigen/LinearAlgebra.hpp>
#include <controlit/Task.hpp>


//#include <controlit/ConstraintSet.hpp>

// hmm...
// #include <Eigen/LU>
// #include <Eigen/SVD>
#include <math.h>
#include <sys/time.h>

#include <iomanip>  // For printing full floating point precision,
                    // see std::setprecision below

// For publishing the equivalent embedded damping gains
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <sstream>
#include <controlit/utility/string_utility.hpp>

#include <controlit/addons/eigen/LinearAlgebra.hpp>

namespace controlit {
namespace controller_library {

using controlit::addons::eigen::Vector;
using controlit::addons::eigen::Matrix;

// Uncomment one of the following lines to enable/disable detailed debug statements.
#define PRINT_DEBUG_STATEMENT(ss)
// #define PRINT_DEBUG_STATEMENT(ss) CONTROLIT_DEBUG << ss;

#define PUBLISH_EMBEDDED_GAINS_THROTTLE_FACTOR 100

WBOSC_Impedance::WBOSC_Impedance()
    : Controller("Anonymous"),
      controlitParameters(nullptr)
{
    torqueController.reset(new WBOSC("Anonymous"));
}

WBOSC_Impedance::WBOSC_Impedance(std::string const& name)
    : Controller(name),
      controlitParameters(nullptr)
{
    torqueController.reset(new WBOSC(name));
}

bool WBOSC_Impedance::init(ros::NodeHandle & nh, ControlModel & model,
    controlit::utility::ControlItParameters * controlitParameters,
    std::shared_ptr<Timer> timer)
{
    // Ensure this WBOSC_Impedance is initialized once
    assert(!initialized);
  
    // Save the pointer to the object holding ControlIt!'s parameters
    this->controlitParameters = controlitParameters;
  
    if (!reinit(model)) return false;
  
    int numDOFs = model.getNActuableDOFs();
  
    // Initialize the size of member variables
    kp.setZero(numDOFs);
    kd.setZero(numDOFs);
  
    qi.setZero(numDOFs);
    qi_prev.setZero(numDOFs);
    qi_dot.setZero(numDOFs);
    qi_dot_prev.setZero(numDOFs);
    qi_ddot.setZero(numDOFs);
    model.constraints().getU(U);
  
    qi_prev = model.getLatestJointState()->getJointPosition();
    qi_dot_prev = model.getLatestJointState()->getJointVelocity();
    prevUpdateTime = ros::Time::now();
  
    // get parameters from ros param
    nh.param<double>("controlit/WBOSC_Impedance/alpha", alpha, 0.5);
    nh.param<double>("controlit/WBOSC_Impedance/beta", beta, 0.1);
    
    // HACK HACK HACK HACK HACK
    modelInitHackParameter = false;
    // HACK HACK HACK HACK HACK
  
    initialized = true;
  
    // Initialize the torque controller
    return torqueController->init(nh, model, controlitParameters, timer);
}

bool WBOSC_Impedance::reinit(ControlModel & model)
{
    qi_prev = model.getLatestJointState()->getJointPosition();
    qi_dot_prev = model.getLatestJointState()->getJointVelocity();
    prevUpdateTime = ros::Time::now();
  
    return torqueController->reinit(model);
}

bool WBOSC_Impedance::computeCommand(ControlModel & model, CompoundTask & compoundTask, Command & command)
{
    if (!torqueController->computeCommand(model, compoundTask, command)) return false;
  
    // Get references to various useful matricies and vector.
    const Matrix & UNcBar = model.constraints().getUNcBar();
    const Matrix & U = model.constraints().getU();
    const Matrix & UNcAiNorm = model.constraints().getUNcAiNorm();
    const Vector & gravityComp = torqueController->getGravityComp();
  
    Matrix Jstar = U * UNcBar;
    Matrix inverseLstar = Jstar * UNcAiNorm * Jstar.transpose();
  
    // forward dyanmics step to turn torque into acceleration
    qi_ddot = inverseLstar * (command.getEffortCmd() - gravityComp); //hmmmm....what if command already includes internal tensions?
  
    // Internal velocity update
    ros::Time currTime = ros::Time::now();
    double dt = (currTime - prevUpdateTime).toSec();
    int numDOFs = model.getNActuableDOFs();
  
    // get current state
    const Vector & qa = model.getLatestJointState()->getJointPosition();
    const Vector & qa_dot = model.getLatestJointState()->getJointVelocity();
  
    // HACK HACK HACK
    if (modelInitHackParameter == false) {
        qi_prev = model.getLatestJointState()->getJointPosition();
        qi_dot_prev = model.getLatestJointState()->getJointVelocity();
        modelInitHackParameter = true;
    }
    // HACK HACK HACK
  
    // compute command update
    q  = qi_prev + dt * qi_dot_prev;
    qd = qi_dot_prev + dt * qi_ddot;
  
    // anti windup stuff
    modelAntiWindup(model, qa);
  
    // Internal position update
    qi = alpha * q + (1.0 - alpha) * qa;
    qi_dot = alpha * qd + (1.0 - alpha) * qa_dot;
  
    PRINT_DEBUG_STATEMENT("Updated internal model:\n"
      " - dt = " << dt << "\n"
      " - qa = " << qa.transpose() << "\n"
      " - qa_dot = " << qa_dot.transpose() << "\n"
      " - qi = " << qi.transpose() << "\n"
      " - qi_dot = " << qi_dot.transpose() << "\n"
      " - qi_ddot = " << qi_ddot.transpose())
  
    // Memory of current state used for next update cycle
    qi_dot_prev = qi_dot;
    qi_prev = qi;
    prevUpdateTime = currTime;
  
    // sanity check on model
    assert(qi.size() == numDOFs);
    assert(qi_dot.size() == numDOFs);
  
    for (int ii = 0; ii < numDOFs; ii++)
    {
        //command.getEffortCmd()[ii] = gravityComp[ii];// + integralTerm[ii];
        command.getPositionCmd()[ii] = qi[ii]; //q_cmd_filter[ii]->process(qi[ii]);
        command.getVelocityCmd()[ii] = qi_dot[ii]; //qd_cmd_filter[ii]->process(qi_dot[ii]);
    }
  
    // For the joints in the gains passthrough mask, simply set the
    // embedded PD gains equal to kp/kd (i.e., do not multiple kp/kd
    // by the mass matrix).
    // const std::vector<std::string> & gainsPassthroughMask = controlitParameters->getEffectiveGainPassThroughMask();
    // for (size_t ii = 0; ii < gainsPassthroughMask.size(); ii++)
    // {
    //   unsigned int index;
    //   if (!model.getJointIndex(gainsPassthroughMask[ii], index)) return false;
  
    //   command.getEffectiveKp()[index] = kp[index];
    //   command.getEffectiveKd()[index] = kd[index];
    // }
  
    // PRINT_DEBUG_STATEMENT("Done method call:\n"
    //      " - command = " << command.transpose());
  
    return true;
}

void WBOSC_Impedance::modelAntiWindup(ControlModel & model, const Vector & qa) 
{
    int numDOFs = model.getNActuableDOFs();
  
    assert(qi.size() == numDOFs);
    assert(qi_dot.size() == numDOFs);
    assert(qi_prev.size() == numDOFs);
    assert(qi_dot_prev.size() == numDOFs);
    assert(qi_ddot.size() == numDOFs);
    assert(qa.size() == numDOFs);
    assert(q.size() == numDOFs);
    assert(qd.size() == numDOFs);
  
    for (int ii = 0; ii < numDOFs; ii++)
    {
        // filter small accelerations
        if(fabs(qi_ddot[ii]) < 1e-1) 
        {
            qd[ii] = qi_dot_prev[ii];
            q[ii] = qi_prev[ii];
            continue;
        }
        if( (qi[ii] - qa[ii]) > beta) 
        {
            // model is greater than actual
            if(qi_ddot[ii] > 0.0) 
            {
                // acceleration is positive, clamp velocity
                qd[ii] = qi_dot_prev[ii];
                q[ii] = qi_prev[ii];
            }
        } 
        else if( (qi[ii] - qa[ii]) < -beta) 
        {
            // model is smaller than actual
            if(qi_ddot[ii] < 0.0) 
            {
                // acceleration is negative, clamp velocity
                qd[ii] = qi_dot_prev[ii];
                q[ii] = qi_prev[ii];
            }
        }
    }
}

// void WBOSC_Impedance::dbg(std::ostream& os,
//     std::string const& title,
//     std::string const& prefix) const
// {
//     torqueController->dbg(os, title, prefix);
// }

} // namespace controller_library
} // namespace controlit
