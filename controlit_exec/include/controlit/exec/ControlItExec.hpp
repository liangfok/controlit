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

#ifndef __CONTROLIT_EXEC_MAIN_HPP__
#define __CONTROLIT_EXEC_MAIN_HPP__

#include "ros/ros.h"
#include <controlit/Coordinator.hpp>

namespace controlit {
namespace exec {

/*!
 * A class that provides the main method for launching a ControlIt!
 * controller.
 */
class ControlItExec
{
public:
    /*!
     * The default constructor.
     */
    ControlItExec();

    /*!
     * The destructor.
     */
    virtual ~ControlItExec();

    /*!
     * Initializes the ControlIt! controller.
     *
     * \return Whether the initialization was successful.
     */
    bool init();

    /*!
     * Starts the ControlIt! controller.
     */
    bool start();

    /*!
     * Stops the ControlIt! controller.
     */
    bool stop();

    /*!
     * Returns a string representation of this class.
     */
    std::string toString(std::string const & prefix = "") const;

private:

    /*!
     * Whether the instantiation of this class is initialized.
     */
    bool initialized;

    /*!
     * The name of the controller.  All parameters for the controller
     * should reside on the ROS parameter server under a namespace
     * with the same name as this controller.
     */
    // std::string controllerName;

    /*!
     * The coordinator that implements the ControlIt! controller.
     */
    // std::unique_ptr<controlit::Coordinator> coordinator;
     controlit::Coordinator coordinator;
};

} // namespace exec
} // namespace controlit

#endif
