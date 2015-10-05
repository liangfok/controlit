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

#ifndef __CONTROLIT_TASK_LIBRARY_ORIENT_VECTOR_TO_VECTOR_TASK_HPP__
#define __CONTROLIT_TASK_LIBRARY_ORIENT_VECTOR_TO_VECTOR_TASK_HPP__

#include <controlit/LatchedTask.hpp>
#include <controlit/task_library/PDController.hpp>
#include <controlit/addons/ros/RealTimePublisher.hpp>
#include <controlit/addons/eigen/LinearAlgebra.hpp>

#include <visualization_msgs/MarkerArray.h>

namespace controlit {
namespace task_library {

using controlit::addons::eigen::Vector;
using controlit::addons::eigen::Matrix;
using controlit::addons::eigen::Vector3d;
using controlit::addons::eigen::Matrix3d;

/*!
 * Controls the orientation of a body on the robot.  The inputs are
 * two vectors: a current heading and a goal heading.  The current
 * heading is defined in the coordinate frame of the body that is being
 * controlled.  The goal heading can be defined in any coordinate
 * frame including "world".  A PD controller is used to rotate the body
 * such that the current heading coincides with the goal heading.
 */
class OrientVectorToVectorTask : public controlit::LatchedTask
{
public:

    /*!
     * The constructor.
     */
    OrientVectorToVectorTask();

    /*!
     * Initializes this task.
     *
     * \param[in] model The control model used to execute this task.
     */
    virtual bool init(ControlModel & model);

    /*!
     * Computes the current orientation.
     */
    virtual bool sense(ControlModel & model);

    /*!
     * Computes the desired commands.
     */
    virtual bool getCommand(ControlModel& model, TaskCommand & command);

protected:
    /*!
     * Overrides the super class' method.  An implementation of the updateState method.
     *
     * \param[in] model The robot's current active control model.
     * Note that this pointer should <b>not</b> be stored as a member
     * variable because it may not be the active one.
     * \param[in] taskState The TaskState that should be updated.
     * \return Whether the update state operation was successful.
     */
    virtual bool updateStateImpl(ControlModel * model, TaskState * taskState);

    /*!
     * The name of the body whose coordinate frame is used to define
     * the current heading vector.
     */
    std::string bodyName_;

    /*!
     * The RBDL ID of the body whose heading is being controlled.
     */
    unsigned int bodyId_;

    Vector goalVector_; // Which defines a vector to whic the bodyFrameVector_ will be aligned
    Vector bodyFrameVector_; //Vector in body frame to align to the goalVector_ in the specified frame

    /*!
     * A PD controller
     */
    std::unique_ptr<PDController> controller;

private:

    void addDefaultBindings();

    Matrix JwBody, JwFrame, Rframe, Jtloc;

    /*
     * A rotation matrix that converts from the world coordinate frame into the
     * body's coordinate frame.
     */
    Matrix Rbody;

    /*!
     * The actual heading in the world coordinate frame.
     */
    Vector actualHeading;

    /*!
     * The goal heading in the world coordinate frame.
     */
    Vector goalHeading;

    /*!
     * The heading error (goal heading - current heading).
     */
    Vector e0;

    /*!
     * Used to publish the goal and current 2D Orientation markers.
     */
    controlit::addons::ros::RealtimePublisher<visualization_msgs::MarkerArray>
        visualizationPublisher;

    /*!
     * The topic on which to publish the marker array messages for visualizing the
     * reference vector and current vector.
     */
    std::string markerTopic;

    /*!
     * The angle between the goal heading and the current heading.
     */
    double errorAngle;

    /*!
     * Persistent pointers to parameters.  These pointers are maintained to prevent
     * having to call lookupParameter(...) every time getCommand(...) is called.
     */
    controlit::Parameter * paramErrorAngle;
    controlit::Parameter * paramGoalHeading;
    controlit::Parameter * paramActualHeading;

    // Variables used by sense()
    Vector Q, Qd;

    Vector base_vector;
    Vector tran_BodyToBase;
};

} // namespace task_library
} // namespace controlit

#endif // __CONTROLIT_TASK_LIBRARY_ORIENT_VECTOR_TO_VECTOR_TASK_HPP__
