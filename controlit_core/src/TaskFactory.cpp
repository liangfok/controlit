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

#include <controlit/TaskFactory.hpp>
#include <controlit/logging/RealTimeLogging.hpp>

namespace controlit {

TaskFactory::TaskFactory() : controlit::YamlFactory<TaskFactory,Task>()
{
    classLoader.reset( new pluginlib::ClassLoader<Task>( "controlit_core", "controlit::Task" ) );
}

Task* TaskFactory::loadFromYamlImpl(YAML::Node const& doc)
{
    std::string taskType;
    doc["type"] >> taskType;

    Task* task = classLoader->createUnmanagedInstance(taskType);

    if (task == NULL)
    {
        CONTROLIT_ERROR << "Pluginlib's class loader failed to load task '" << taskType << "'. Aborting task creation.";
        return NULL;
    }

    if (!task->loadConfig(doc))
    {
        CONTROLIT_ERROR << "Task of type '" << taskType << "' failed to load config. Aborting task creation.";
        return NULL;
    }

    // CONTROLIT_INFO << "Created task of type '" << taskType << "'";
    return task;
}

}  // namespace controlit
