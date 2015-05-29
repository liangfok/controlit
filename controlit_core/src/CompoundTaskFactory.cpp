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

#include <controlit/logging/RealTimeLogging.hpp>

#include <controlit/CompoundTaskFactory.hpp>

namespace controlit {

CompoundTaskFactory::CompoundTaskFactory() :
  controlit::YamlFactory<CompoundTaskFactory,CompoundTask>()
{}

CompoundTask* CompoundTaskFactory::loadFromYamlImpl(YAML::Node const& doc)
{
    // Peek into 'compound_task' section to get type and name.
    // This feels naughty (single level of abstraction). Name should probably be
    // an argument to the factory. Type maybe as well? Ticket created: DRC-292
    YAML::Node const* ctNode = doc.FindValue("compound_task");
    if (ctNode != NULL)
    {
        std::string name, type;
        (*ctNode)["type"] >> type;
    
        CompoundTask* ct(new CompoundTask);
        if (!ct->loadConfig(doc))
        {
            CONTROLIT_ERROR << "Compound task of type '" << type << "' failed to load config. Aborting compound task creation.";
            return NULL;
        }
    
        // CONTROLIT_INFO << "Created compound task of type '" << type << "'";
    
        return ct;
    }
  
    CONTROLIT_ERROR << "Unable to find 'compound_task' in YAML config. Aborting compound task creation.";
    return NULL;
}

} // namespace controlit
