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

#include <controlit/BindingManager.hpp>

#include <controlit/ParameterReflection.hpp>

namespace controlit {

// Uncomment one of the following lines to enable/disable detailed debug statements.
#define PRINT_DEBUG_STATEMENT(ss)
// #define PRINT_DEBUG_STATEMENT(ss) CONTROLIT_DEBUG << ss;


BindingManager::BindingManager()
{
    classLoader.reset(new pluginlib::ClassLoader<BindingFactory>( "controlit_core", "controlit::BindingFactory"));
}


BindingManager::~BindingManager()
{
    PRINT_DEBUG_STATEMENT("Method called!");
}

bool BindingManager::init(ros::NodeHandle & nh)
{
    // Create a ROS parameter accessor
    paramAccessor.reset(new controlit::addons::ros::ROSParameterAccessor(nh));

    // Load the names of the binding factory plugins
    std::vector<std::string> * bindingFactoryNamesPtr = nullptr;

    if (!paramAccessor->loadParameter("controlit/parameter_binding_factories", &bindingFactoryNamesPtr))
    {
        CONTROLIT_WARN << "Parameter '" << nh.getNamespace() << "/parameter_binding_factories' not set!  "
                   << "No parameter binding factories loaded.";
        return false;
    }

    // Load the binding factory plugins
    for(std::vector<std::string>::iterator it = bindingFactoryNamesPtr->begin(); it != bindingFactoryNamesPtr->end(); ++it)
    {
        std::string bindingFactoryName = *it;

        PRINT_DEBUG_STATEMENT("Creating BindingFactory \"" << bindingFactoryName << "\"...");
        std::unique_ptr<BindingFactory> bfPtr(nullptr);
        bfPtr.reset(classLoader->createUnmanagedInstance(bindingFactoryName));

        if (bfPtr.get() == nullptr)
        {
            CONTROLIT_ERROR << "Failed to create parameter binding factory \"" << bindingFactoryName << "\"!";
            return false;
        }

        bindingFactories.push_back(std::move(bfPtr));
    }

    return true;
}

bool BindingManager::bindParameters(ros::NodeHandle & nh, controlit::ParameterReflection & pr)
{
    // Go through each of the binding config parameters within the supplied parameter
    // reflection object and attempt to bind it using the binding factories managed
    // by this object.  Abort if any of the binding creations fail.

    for (auto const & bindingConfigParam : pr.lookupParameters(controlit::PARAMETER_TYPE_BINDING))
    {
        controlit::BindingConfig const & config = *(bindingConfigParam->getBindingConfig());

        std::string parameterName = config.getParameter();

        controlit::Parameter * param = pr.lookupParameter(parameterName);
        if (param == nullptr)
        {
            CONTROLIT_WARN << "Binding configuration referenced non-existent parameter \""
                << pr.getTypeName() << "." << pr.getInstanceName() << "." << parameterName << "\"! Aborting parameter binding process.";
            return false;
        }

        bool paramBound = false;

        for (auto & bindingFactory : bindingFactories)
        {
            Binding * binding = bindingFactory->createBinding(nh, config, param);
            if (binding != nullptr)
            {
                PRINT_DEBUG_STATEMENT("Successfully created binding for parameter:\n"
                    << "  - In ParameterReflection object: " << pr.getTypeName() << "." << pr.getInstanceName() << "\n"
                    << "  - Name of parameter: " << param->name() << "\n"
                    << "  - Memory address of parameter: " << param << "\n"
                    << "  - Memory address of binding: " << binding);

                // Save the newly created binding in the 'bindings' list.
                bindings.push_back(std::unique_ptr<Binding>(binding));

                paramBound = true;
                break;
            }
        }

        if (!paramBound)
        {
            CONTROLIT_ERROR 
                << "Failed to bind parameter!\n"
                << "  - type name: " << pr.getTypeName() << "\n"
                << "  - instance name: " << pr.getInstanceName() << "\n"
                << "  - parameter name: " << param->name() << "\n"
                << "  - " << config.toString("    ");
            return false;
        }
        
    }

    return true;
}

bool BindingManager::unbindParameters(ParameterReflection & pr)
{
    // Go through each of the binding config parameters within the supplied parameter
    // reflection object and attempt to unbind it using the binding factories managed
    // by this object.  Abort if any of the binding destructions fail.

    for (auto const & bindingConfigParam : pr.lookupParameters(controlit::PARAMETER_TYPE_BINDING))
    {
        controlit::BindingConfig const & config = *(bindingConfigParam->getBindingConfig());

        std::string parameterName = config.getParameter();

        controlit::Parameter * param = pr.lookupParameter(parameterName);
        if (param != nullptr)
        {
            bool bindingDestroyed = false;

            for (std::list<std::unique_ptr<Binding>>::iterator iterator = bindings.begin(), end = bindings.end();
                !bindingDestroyed && iterator != end; ++iterator)
            {
                if ((*iterator)->isForParameter(param))
                {
                    // Binding * bptr = (*iterator).get();

                    // PRINT_DEBUG_STATEMENT("Unbinding parameter:\n"
                    //     << "  - In ParameterReflection object: " << pr.getTypeName() << "." << pr.getInstanceName() << "\n"
                    //     << "  - Name of parameter: " << param->name() << "\n"
                    //     << "  - Memory address of parameter: " << param << "\n"
                    //     << "  - Memory address of binding: " << bptr << "\n"
                    //     << "  - Binding: " << *bptr);

                    bindings.erase(iterator);

                    PRINT_DEBUG_STATEMENT("Successfully unbound parameter");

                    bindingDestroyed = true;
                    break;
                }
            }

            if (!bindingDestroyed)
            {
                CONTROLIT_ERROR << "Failed to unbind parameter \"" << pr.getTypeName() << "." << pr.getInstanceName() << "." << param->name() <<"\"!\n"
                             << "  - BindingConfig:\n" << config.toString("    ");
                return false;
            }
        }
        else
        {
            CONTROLIT_WARN << "Binding configuration referenced non-existent parameter \""
                       << pr.getTypeName() << "." << pr.getInstanceName() << "." << parameterName << "\"!";
        }
    }

    return true;
}

} // namespace controlit
