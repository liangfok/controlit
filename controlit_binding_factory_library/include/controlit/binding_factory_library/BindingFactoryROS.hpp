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

#ifndef __CONTROLIT_BINDING_FACTORY_LIBRARY_BINDING_FACTORY_ROS_HPP__
#define __CONTROLIT_BINDING_FACTORY_LIBRARY_BINDING_FACTORY_ROS_HPP__

#include <controlit/BindingFactory.hpp>

namespace controlit {
namespace binding_factory_library {

/*!
 * A binding factory that produces bindings between ControlIt! parameters
 * and ROS topics.
 */
class BindingFactoryROS : public controlit::BindingFactory
{
public:
    /*!
     * The constructor.
     */
    BindingFactoryROS();

    /*!
     * The destructor.
     */
    virtual ~BindingFactoryROS();

    /*!
     * Creates a binding of the specified type.  This method may return nullptr if
     * it cannot create the binding.
     *
     * \param[in] nh The ROS node handle to use.
     * \param[in] config The binding configuration.  This contains transport layer parameters.
     * \param[in] param The parameter being bound.
     * \return A pointer to the newly created Binding or nullptr if this factory cannot create the requested binding.
     */
    Binding * createBinding(ros::NodeHandle & nh, const BindingConfig & config,
        Parameter * param);

    /*!
     * Destroys a binding.
     *
     * \param[in] config The binding configuration.  This contains transport layer parameters.
     * \param[in] param The parameter being bound.
     * \return true if the binding was successfully destroyed.
     */
    // bool destroyBinding(ros::NodeHandle & nh, const BindingConfig & config,
    //     Parameter * param);

protected:
    /*!
     * Creates an input binding of the specified type.  This method may return nullptr if
     * it cannot create the binding.
     *
     * \param[in] nh The ROS node handle to use.
     * \param[in] config The binding configuration.  This contains transport layer parameters.
     * \param[in] param The parameter being bound.
     * \return A pointer to the newly created Binding or nullptr if this factory cannot create the requested binding.
     */
    Binding * createInputBinding(ros::NodeHandle & nh,
        const controlit::BindingConfig & config, controlit::Parameter * param);

    /*!
     * Creates an output binding of the specified type.  This method may return nullptr if
     * it cannot create the binding.
     *
     * \param[in] nh The ROS node handle to use.
     * \param[in] config The binding configuration.  This contains transport layer parameters.
     * \param[in] param The parameter being bound.
     * \return A pointer to the newly created Binding or nullptr if this factory cannot create the requested binding.
     */
    Binding * createOutputBinding(ros::NodeHandle & nh,
        const controlit::BindingConfig & config, controlit::Parameter * param);
};

} // namespace binding_factory_library
} // namespace controlit

#endif // __CONTROLIT_BINDING_FACTORY_LIBRARY_BINDING_FACTORY_ROS_HPP__
