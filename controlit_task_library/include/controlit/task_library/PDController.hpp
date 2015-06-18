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

#ifndef __CONTROLIT_TASK_LIBRARY_PD_CONTROLLER_HPP__
#define __CONTROLIT_TASK_LIBRARY_PD_CONTROLLER_HPP__

#include <controlit/ParameterReflection.hpp>
#include <controlit/addons/eigen/LinearAlgebra.hpp>
#include <controlit/task_library/SaturationPolicy.hpp>
#include <memory>
#include <controlit/addons/eigen/LinearAlgebra.hpp>

namespace controlit {
namespace task_library {

// Forward declare
class ParameterReflection;

using controlit::addons::eigen::Vector;
using controlit::addons::eigen::Matrix;
using controlit::addons::eigen::Vector3d;
using controlit::addons::eigen::Matrix3d;

/* ***************************************************************************
  Base PDController class
 * **************************************************************************/
struct PDController
{
    /*!
     * Declare the PD controller parameters.
     *
     * \param[in] pr The parameter reflection object that owns the
     * parameters.
     */
    virtual void declareParameters(controlit::ParameterReflection* pr) = 0;

    /*!
     * Resizes internal state vectors to be of the correct dimension.
     *
     * \param dimension The desired size of the vectors.
     * \param initDefault Whether to initialize incorrectly-sized
     * parameters to default values (default is false).
     * \return Whether the resize operation was successful.
     */
    virtual bool resize(int dimension, bool initDefault = false) = 0;

    /*!
     * Computes the next command based on the current state.
     *
     * \param xd The desired joint state.
     * \param x The current joint state.
     * \param xd_dot The desired joint velocity.
     * \param x_dot The current joint velocity.
     * \param u This is where the next command is saved.
     * \param pr A pointer to the ParameterReflection object that is executing this method.
     * \return Whether the command was successfully computed.
     */
    virtual bool computeCommand(Vector const& xd, Vector const& x,
           Vector const& xd_dot, Vector const& x_dot,
           Vector& u, controlit::ParameterReflection* pr) = 0;

    /*!
     * Computes the next command based on the current state.
     *
     * \param e The current position error.
     * \param e_dot The current velocity error.
     * \param u The resulting command.
     * \param pr A pointer to the ParameterReflection object that is executing this method.
     * \return Whether the command was successfully computed.
     */
    virtual bool computeCommand(Vector const& e, Vector const& e_dot, Vector& u,
        controlit::ParameterReflection* pr) = 0;
};

/* ***************************************************************************
  PDController Factory
 * **************************************************************************/
struct PDControllerFactory
{
    static PDController* create(SaturationPolicy::Options policy);
};

} // namespace task_library
} // namespace controlit

#endif