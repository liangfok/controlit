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

#include <controlit/LatchedTask.hpp>

namespace controlit {

LatchedTask::LatchedTask() :
    Task("__UNNAMED_LATCHED_TASK__", CommandType::ACCELERATION, nullptr, nullptr),
    frameName_(""),
    frameId_(std::numeric_limits<unsigned int>::max()),
    latchOn_(0),
    resetLatch_(0),
    paramLatchOn(NULL),
    paramResetLatch(NULL),
    isLatched(0)
{
    setupParameters();
}

LatchedTask::LatchedTask(std::string const& typeName, CommandType commandType, TaskState * activeState, TaskState * inactiveState) :
    Task(typeName, commandType, activeState, inactiveState),
    frameName_(""),
    frameId_(std::numeric_limits<unsigned int>::max()),
    latchOn_(0),
    resetLatch_(0),
    paramLatchOn(NULL),
    paramResetLatch(NULL),
    isLatched(0)
{
    setupParameters();
}

void LatchedTask::setupParameters()
{
    paramLatchOn = declareParameter("latchOn", & latchOn_);
    paramResetLatch = declareParameter("resetLatch", & resetLatch_);
    declareParameter("frameName", & frameName_);
}

bool LatchedTask::init(const ControlModel & model)
{
    // Get the reference frame's RBDL id from the name, assume world reference if not specified
    if (!model.getFrameID(frameName_, frameId_))
    {
        CONTROLIT_PR_ERROR << "Problems getting frameID using frameName '" << frameName_ << "'";
        return false;
    }
    return true;
}

void LatchedTask::updateLatch(ControlModel * model)
{
    //The latch has just been turned on, grab the current frame
    if(!isLatched && latchOn_)
    {
        latchedTranslation = RigidBodyDynamics::CalcBodyToBaseCoordinates(model->rbdlModel(), model->getQ(), frameId_, Vector::Zero(3), true);
        latchedRotation  = RigidBodyDynamics::CalcBodyWorldOrientation(model->rbdlModel(), model->getQ(), frameId_, false);
        isLatched = true;
        return;
    }
  
    //The latch has just been turned off....go back to moving frame
    if(isLatched && !latchOn_)
    {
        isLatched = false;
        return;
    }
  
    //The latch should be reset (and turned on) to the current frame.
    if(resetLatch_)
    {
        latchedTranslation = RigidBodyDynamics::CalcBodyToBaseCoordinates(model->rbdlModel(), model->getQ(), frameId_, Vector::Zero(3), true);
        latchedRotation  = RigidBodyDynamics::CalcBodyWorldOrientation(model->rbdlModel(), model->getQ(), frameId_, false);
        // Just in case
        isLatched = true;
        paramLatchOn->set(1);
        // Set to false so that reset can be run again
        paramResetLatch->set(0);
    }
}

} // namespace controlit
