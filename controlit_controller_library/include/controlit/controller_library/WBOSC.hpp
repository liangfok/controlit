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

#ifndef __CONTROLIT_WBOSC_CONTROLLER_HPP__
#define __CONTROLIT_WBOSC_CONTROLLER_HPP__

#include <controlit/Controller.hpp>
#include <controlit/Timer.hpp>
#include <controlit/utility/ContainerUtility.hpp>
#include <controlit/utility/GravityCompensationPublisher.hpp>

#include "controlit_core/get_parameters.h"

namespace controlit {
namespace controller_library {

/*!
 * Implements a whole body operational space controller.
 */
class WBOSC : public Controller
{
public:
    /*!
     * The default constructor. This is necessary to be compatible with pluginlib.
     *
     */
    explicit WBOSC();
    
    /*!
     * The constructor.
     *
     * \param[in] name The name of this controller.  This can be any string.
     * Preferably it should be human-readable.
     */
    explicit WBOSC(std::string const & name);
  
    /*!
     * Computes the command.
     *
     * \param[in] model The robot's control model.
     * \param[in] compoundTask The compound task being executed.
     * \param[out] command Where the resulting command should be stored.
     * \return Whether the command was successfully computed.
     */
    virtual bool computeCommand(ControlModel & model, CompoundTask & compoundTask, Command & command);
  
    /*!
     * Initializes this controller.  This should only be called once.
     *
     * \param[in] nh The ROS node handle to be used by this controller.
     * \param[in] model A control model that defines certain properties
     * of the robot being controlled like the number of joints.
     * Note that this controller should *not* keep a reference to this model
     * as the active ControlModel will change over time.
     * \param[in] controlitParameters A pointer to the object holding the WBC parameters.
     * \param[in] timer A timer for measuring the internal latencies of computeCommand(...).
     * \return Whether the initialization was successful.
     */
    virtual bool init(ros::NodeHandle & nh, ControlModel & model,
      controlit::utility::ControlItParameters * controlitParameters,
      std::shared_ptr<Timer> timer);
  
    /*!
     * Re-Initializes this controller.  This can be called any number of times
     * after init is called.
     *
     * \param[in] model A control model that defines certain properties
     * of the robot being controlled like the number of joints.
     * Note that this controller should *not* keep a reference to this model
     * as the active ControlModel will change over time.
     * \return Whether the initialization was successful.
     */
    virtual bool reinit(ControlModel & model);
  
    /*!
     * Obtains the gravity compensation vector.
     *
     * \return A reference to the gravity compensation vector.
     */
    const Vector & getGravityComp() { return gravityComp; }
  
    /*!
     * Prints a string description of this class to the supplied output
     * stream.  This is useful for debugging.
     *
     * \param[in] os The output stream to which to write the string
     * description.
     * \param[in] title The title to print at the very beginning of the string.
     * \param[in] prefix A prefix to place at the beginning of each line
     * in the string.
     */
    // virtual void dbg(std::ostream& os,
    //                  std::string const& title,
    //                  std::string const& prefix) const;

protected:
    std::shared_ptr<Timer> timer;

private:

    // The following variables are used to compute the internal latencies
    // of a call to computeCommand(...).
    double timeBookkeeping, timeGetJacobianAndCommand, timeAddGravityComp,
        timeCheckMagnitude, timeCheckForceTaskEnabled, timeAddVLM,
        timeCheckMagnitude2;

    /*!
     * Defines the threshold above which a value is considered to be infinity.
     * This is used in checking the validity of various data structures in WBC.
     */
    #define INFINITY_THRESHOLD 1e10
  
    /*!
     * The WBC parameters.
     */
    controlit::utility::ControlItParameters * controlitParameters;
  
    /*!
     * Ensures the matrices used in the controller's computations remain valid.
     */
    controlit::utility::ContainerUtility containerUtility;
  
    /*!
     * An identity matrix of size # actuable DOFs x # actuable DOFs.
     */
    Matrix identityActuableDOFs;
  
    /*!
     * The null space of all higher priority tasks.
     * Its dimensions are # actuable DOFs x # actuable DOFs.
     */
    Matrix Nhp;
  
    /*!
     * The task-level gravity compensation term.
     * Its dimension is # actuable DOFs.
     */
    Vector pstar;
  
    /*!
     * The effort from higher-priority tasks.
     * Its dimension is # actuable DOFs.
     */
    Vector fcomp;
  
    /*!
     * The gravity compensation vector.  This is the force or torque
     * applied to each joint to compensate for the effects of gravity.
     */
    Vector gravityComp;
  
    /*!
     * Used to publish the gravity compensation vector.
     */
    controlit::utility::GravityCompensationPublisher gravityCompensationPublisher;
  
    /*!
     * The operational space mass/inertia.
     */
    Matrix Lstar;
  
    // These are used to prevent dynamic allocation when including
    // virtual linkage model commands
    Matrix Jlstarbar;
    Matrix Jlstar;
    Vector pl;
    Vector Fint;
    Vector FintRef;
};

} // namespace controller_library
} // namespace controlit

#endif // __CONTROLIT_WBOSC_CONTROLLER_HPP__
