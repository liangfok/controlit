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

#ifndef __CONTROLIT_CORE_BINDING_MANAGER_HPP__
#define __CONTROLIT_CORE_BINDING_MANAGER_HPP__


#include <memory>
#include <list>

#include <ros/ros.h>
#include <pluginlib/class_loader.h>

#include <controlit/addons/ros/ROSParameterAccessor.hpp>
#include <controlit/BindingFactory.hpp>
#include <controlit/ParameterReflection.hpp>

namespace controlit {

using controlit::addons::ros::ROSParameterAccessor;

/*!
 * Handles the binding of parameters to various transport layers.
 * It maintains a list of binding factories searches this list
 * for a factory that can create a particular binding.
 */
class BindingManager
{
public:

    /*!
     * The constructor.
     *
     */
    BindingManager();

    /*!
     * The destructor.
     */
    ~BindingManager();

    /*!
     * Initializes this parameter binding manager.  This takes a list of
     * binding factories from the ROS parameter server, instantiates them using
     * pluginlib, and stores them in the bindingFactories list.
     *
     * \param[in] nh The ROS node handle to use during the initialization process.
     * \return Whether the initialization was successful.
     */
    virtual bool init(ros::NodeHandle & nh);

    /*!
     * Sets up all of the bindings stored within the ParameterReflection object.
     *
     * \param[in] nh The node handle to use.
     * \param[in] pr The ParameterReflection object containing the parameters to be bound.
     * \return Whether the bindings were successfully created.
     */
    virtual bool bindParameters(ros::NodeHandle & nh, ParameterReflection & pr);

    /*!
     * Destroys the bindings stored in the ParameterReflection object.
     *
     * \param[in] pr The ParameterReflection object containing the parameters to be unbound.
     * \return Whether the bindings were successfully destroyed.
     */
    virtual bool unbindParameters(ParameterReflection & pr);

private:

    /*!
     * The class loader that is used to create the BindingFactory objects.
     */
    std::unique_ptr<pluginlib::ClassLoader<BindingFactory>> classLoader;

    /*!
     * This is used to fetch parameters from the ROS parameter server.
     */
    std::unique_ptr<ROSParameterAccessor> paramAccessor;

    /*!
     * A list of available binding factories.
     */
    std::list<std::unique_ptr<BindingFactory>> bindingFactories;

    /*!
     * A list of all bindings managed by this object.
     */
    std::list<std::unique_ptr<Binding>> bindings;
};

} // namespace controlit

#endif // __CONTROLIT_CORE_BINDING_MANAGER_HPP__
