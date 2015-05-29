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

#ifndef __CONTROLIT_CORE_CONTROLLER_HPP__
#define __CONTROLIT_CORE_CONTROLLER_HPP__

#include <controlit/Command.hpp>
#include <controlit/CompoundTask.hpp>
#include <controlit/ControlModel.hpp>
#include <controlit/Timer.hpp>
#include <controlit/ParameterReflection.hpp>
#include <controlit/utility/ControlItParameters.hpp>

namespace controlit {

/*!
 * The top-level class of all controllers.
 */
class Controller : public ParameterReflection
{
public:
    /*!
     * The constructor.
     *
     * \param name The instance name of this controller.
     */
    explicit Controller(std::string const& name);

    /*!
     * Initializes this controller.
     *
     * \param[in] nh The ROS node handle to be used by this controller.
     * \param[in] model A control model that defines certain properties
     * of the robot being controlled like the number of joints.
     * Note that this controller should *not* keep a reference to this model
     * as the active ControlModel will change over time.
     * \param[in] controlitParameters A pointer to the object holding the WBC parameters.
     * \param[in] timer A timer for measuring the internal latencies of computeCommand(...).
     * \return Whether the initalization was successful.
     */
    virtual bool init(ros::NodeHandle & nh, ControlModel & model,
        controlit::utility::ControlItParameters * controlitParameters,
        std::shared_ptr<Timer> timer) = 0;

    /*!
     * Computes the force command that should be issued.
     * As opposed to the previous implementation,
     * any logic as to which compoundTask should be used to generate the command
     * needs to be done in an outer servo loop
     *
     * \param[in] model The robot model, which has ALREADY been initialized and/or updated with the current state prior to call.
     * \param[in] compoundTask The compound task to perform.
     * \param[out] command Where the resulting command should be stored.
     * \return Whether the command was successfully computed.
     */
    virtual bool computeCommand(ControlModel & model, CompoundTask & compoundTask, Command & command) = 0;
    
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
    // virtual void dbg(std::ostream& os, std::string const& title, std::string const& prefix) const {}

protected:
    /*!
     * Whether init() was called on this controller.
     */
    bool initialized;
};

} // namespace controlit

#endif // __CONTROLIT_CORE_CONTROLLER_HPP__
