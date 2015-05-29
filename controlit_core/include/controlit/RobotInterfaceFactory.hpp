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

#ifndef __CONTROLIT_CORE_ROBOT_INTERFACE_FACTORY_HPP__
#define __CONTROLIT_CORE_ROBOT_INTERFACE_FACTORY_HPP__

#include <memory>
#include <pluginlib/class_loader.h>

#include <controlit/RobotInterface.hpp>

namespace controlit {

/*!
 * A factory for creating pointers to RobotInterface objects.
 */
class RobotInterfaceFactory
{
public:
    /*!
     * The constructor.
     */
    RobotInterfaceFactory();

    /*!
     * Creates a RobotInterface object.
     *
     * \param robotInterfaceName The name of the robot interface plugin to create.
     */
    RobotInterface * create(std::string & robotInterfaceName);

private:

    /*!
     * The class loader that is used to create the RobotInterface object.
     */
    std::unique_ptr<pluginlib::ClassLoader<RobotInterface>> classLoader;
};

} // namespace controlit

#endif // __CONTROLIT_CORE_ROBOT_INTERFACE_FACTORY_HPP__
