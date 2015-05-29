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

#ifndef __CONTROLIT_CORE_BINDING_FACTORY_HPP__
#define __CONTROLIT_CORE_BINDING_FACTORY_HPP__

#include <ros/ros.h>

#include <controlit/Binding.hpp>
#include <controlit/Parameter.hpp>
#include <controlit/BindingConfig.hpp>

namespace controlit {

/*!
 * The parent class of all binding factories.  Child classes implement
 * factories that produce transport/middleware-specific bindings.
 */
class BindingFactory
{
public:
    /*!
     * The constructor.
     */
    BindingFactory();

    /*!
     * The destructor.
     */
    virtual ~BindingFactory();

    /*!
     * Creates a binding of the specified type.  Returns nullptr if
     * the binding cannot be created.
     *
     * \param[in] nh The ROS node handle to use.
     * \param[in] config The binding configuration.  This contains transport layer parameters.
     * \param[in] param The parameter being bound.
     * \return A pointer to the newly created Binding or nullptr if this factory cannot create the requested binding.
     */
    virtual Binding * createBinding(ros::NodeHandle & nh, const BindingConfig & config,
        Parameter * param) = 0;
};

} // namespace controlit

#endif // __CONTROLIT_CORE_BINDING_FACTORY_HPP__
