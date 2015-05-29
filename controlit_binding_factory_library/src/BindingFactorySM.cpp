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

#include <controlit/binding_factory_library/BindingFactorySM.hpp>
#include <controlit/logging/RealTimeLogging.hpp>

namespace controlit {
namespace binding_factory_library {

// Uncomment one of the following lines to enable/disable detailed debug statements.
#define PRINT_DEBUG_STATEMENT(ss)
// #define PRINT_DEBUG_STATEMENT(ss) CONTROLIT_DEBUG << ss;

#define PRINT_DEBUG_STATEMENT_RT(ss)
// #define PRINT_DEBUG_STATEMENT_RT(ss) CONTROLIT_DEBUG_RT << ss;

#define PRINT_INFO_STATEMENT_RT_ALWAYS(ss) CONTROLIT_INFO_RT << ss;
// #define PRINT_INFO_STATEMENT_RT_ALWAYS(ss) std::cout << ss << std::endl;

BindingFactorySM::BindingFactorySM() :
    BindingFactory() // Call super-class' constructor
{
    // PRINT_DEBUG_STATEMENT("BindingFactorySM Created");
}

BindingFactorySM::~BindingFactorySM()
{
}

Binding * BindingFactorySM::createBinding(ros::NodeHandle & nh, const BindingConfig & config,
        Parameter * param)
{
    // PRINT_DEBUG_STATEMENT("Method called!\n"
    //     << "  - config:\n"
    //     << config.toString("     ") << "\n"
    //     << "  - parameter type = " << controlit::Parameter::parameterTypeToString(param->type()));

    if (config.getTransportType() == "SM")
    {
        if (config.getDirection() == controlit::BindingConfig::Input)
            return createInputBinding(nh, config, param);
        else
            return createOutputBinding(nh, config, param);
    }

    return nullptr;
}

Binding * BindingFactorySM::createInputBinding(ros::NodeHandle & nh,
    const controlit::BindingConfig & config, controlit::Parameter * param)
{
    // TODO
    return nullptr;
}

Binding * BindingFactorySM::createOutputBinding(ros::NodeHandle & nh,
    const controlit::BindingConfig & config, controlit::Parameter * param)
{
    // TODO
    return nullptr;
}

} // namespace binding_factory_library
} // namespace controlit
