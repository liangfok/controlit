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

#ifndef __CONTROLIT_WBOSC_IMPEDANCE_CONTROLLER_HPP__
#define __CONTROLIT_WBOSC_IMPEDANCE_CONTROLLER_HPP__

#include <controlit/controller_library/WBOSC.hpp>
#include <controlit/utility/ContainerUtility.hpp>
#include <controlit/utility/GravityCompensationPublisher.hpp>
#include <controlit/addons/eigen/LinearAlgebra.hpp>

namespace controlit {
namespace controller_library {

using controlit::addons::eigen::Vector;
using controlit::addons::eigen::Matrix;
using controlit::addons::eigen::Vector3d;
using controlit::addons::eigen::Matrix3d;

/*!
 * Implements a whole body operational space impedance controller.  It uses the output of WBOSC
 * and then performs forward dynamics and integration to get the impedance commands.
 */
class WBOSC_Impedance : public Controller
{
public:
    /*!
     * The default constructor. This is necessary to be compatible with pluginlib.
     *
     */
    explicit WBOSC_Impedance();

    /*!
     * The constructor.
     *
     * \param[in] name The name of this controller.  This can be any string.
     * Preferably it should be human-readable.
     */
    explicit WBOSC_Impedance(const std::string & name);
  
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
     *  of the robot being controlled like the number of joints.
     *  Note that this controller should *not* keep a reference to this model
     *  as the active ControlModel will change over time.
     * \param[in] controlitParameters A pointer to the object holding the WBC parameters.
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
     * Updates qi and qi_dot model (i.e., eventual command) to make sure
     * it doesnt diverge too much from qa actual
     *
     * \param[in] model A control model that defines certain properties
     * of the robot being controlled like the number of joints.
     * Note that this controller should *not* keep a reference to this model
     * as the active ControlModel will change over time.
    *  \param[in] qa actual robot joint position.
     * \return Whether the initialization was successful.
     */
    virtual void modelAntiWindup(ControlModel & model, const Vector & qa);
  
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
  
private:
    /*!
     * The torque controller.  This generates the joint effort command that is used
     * by this controller to perform impedance control.
     */
    std::unique_ptr<WBOSC> torqueController;
  
    /*!
     * The WBC parameters.
     */
    controlit::utility::ControlItParameters * controlitParameters;
  
    /*!
     * The operational space mass/inertia.
     */
    Matrix Lstar;
  
    /*
     *For use with post-processed JPos controller--should be initialized in init()
     */
    Vector kp;
    Vector kd;
  
    /*!
     * The joint position and velocity internal models.
     */
    Vector qi, qi_prev;
    Vector qi_dot, qi_dot_prev;
    Vector q, qd;
  
    /*!
     * internal model parameters
     */
    double alpha, beta;
  
    /*!
     * The underactuation matrix.  This specifies which joints are actuable.
     */
    Matrix U;
  
    /*!
     * This a problem.
     */
    bool modelInitHackParameter;
  
    /*!
     * The joint accelerations as computed by forward dynamics.
     */
    Vector qi_ddot;
  
    /*!
     * The time step to use when integrating the Qdd.
     */
    ros::Time prevUpdateTime;
  
    /*!
     * cmd filter buffer
     */
    double filter_buffer;

};

} // namespace controller_library
} // namespace controlit

#endif // __CONTROLIT_WBOSC_IMPEDANCE_CONTROLLER_HPP__
