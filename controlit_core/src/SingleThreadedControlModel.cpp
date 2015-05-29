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

#include <controlit/SingleThreadedControlModel.hpp>
#include <controlit/logging/RealTimeLogging.hpp>

namespace controlit {

// Uncomment one of the following lines to enable/disable detailed debug statements.
#define PRINT_DEBUG_STATEMENT(ss)
// #define PRINT_DEBUG_STATEMENT(ss) CONTROLIT_DEBUG_RT << ss;

SingleThreadedControlModel::SingleThreadedControlModel()
{
    PRINT_DEBUG_STATEMENT("Method Called!")
}

SingleThreadedControlModel::~SingleThreadedControlModel()
{
    PRINT_DEBUG_STATEMENT("Method Called!")
}

void SingleThreadedControlModel::setAMask(std::vector<std::vector<std::string>> mask)
{
    activeModel->setAMask(mask);
}

void SingleThreadedControlModel::setGravMask(std::vector<std::string> mask)
{
    activeModel->setGravMask(mask);
}

void SingleThreadedControlModel::startThread()
{
    PRINT_DEBUG_STATEMENT("Method Called!")

    // Don't do anything since this is a single-threaded implementation.
}

void SingleThreadedControlModel::stopThread()
{
    PRINT_DEBUG_STATEMENT("Method Called!")

    // Don't do anything since this is a single-threaded implementation.
}

bool SingleThreadedControlModel::checkUpdate()
{
    PRINT_DEBUG_STATEMENT("Method Called!")

    // Pretend a new ControlModel is always available.
    return true;
}

ControlModel * SingleThreadedControlModel::trylock()
{
    PRINT_DEBUG_STATEMENT("Method Called!")

    // Always return a pointer to the active model.  No need to lock anything
    // since there is only one thread.
    return activeModel;
}

void SingleThreadedControlModel::unlockAndUpdate()
{
    PRINT_DEBUG_STATEMENT("Method Called!")

    // ros::Time startTime = ros::Time::now();

    // Do the actual update!
    activeModel->update();

    // double elapsedTime = (ros::Time::now() - startTime).toSec();

    //PRINT_DEBUG_STATEMENT("Latency of updating model (ms): " << (1000 * elapsedTime))

}

} // namespace controlit
