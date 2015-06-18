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

#ifndef __CONTROLIT_TASK_LIBRARY_ORIENT_QUATERNION_TASK_HPP__
#define __CONTROLIT_TASK_LIBRARY_ORIENT_QUATERNION_TASK_HPP__

#include <controlit/LatchedTask.hpp>
#include <controlit/task_library/PDController.hpp>
#include <controlit/addons/eigen/LinearAlgebra.hpp>
#include <controlit/addons/ros/RealTimePublisher.hpp>

#include <visualization_msgs/MarkerArray.h>

namespace controlit {
namespace task_library {

using controlit::addons::eigen::Vector;
using controlit::addons::eigen::Matrix;
using controlit::addons::eigen::Vector3d;
using controlit::addons::eigen::Matrix3d;

class OrientQuaternionTask : public controlit::LatchedTask
{
public:
    /*!
     * The default constructor.
     */
    OrientQuaternionTask();

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
     *
     */
    std::string bodyName_;

    /*!
     *
     */
    unsigned int bodyId_;

    /*!
     * Goal quaternion in the frameName_ frame
     */
    Vector goalPosition_;

    /*!
     * Controled quaternion in the bodyName_ frame
     */
    Vector controlPoint_;

    /*!
     * Goal angular velocity in the frameName_ frame
     */
    Vector goalVelocity_;

    /*!
     * Actual (control point) quaternion in the frameName_ frame
     */
    Vector actualPosition_;

    /*!
     * Actual quaternion in the world frame
     */
    Vector actualWorldPosition_;

    /*!
     * Error quaternion
     */
    Vector errorQuat_;

    /*!
     * A PD controller
     */
    std::unique_ptr<PDController> controller;

private:
    void addDefaultBindings();

    Eigen::Quaternion<double> goalQuat, cpQuat, frameQuat, bodyQuat, curQuat, curInFrameQuat, desQuat, errQuat;
    Eigen::Quaternion<double>::Matrix3 goalRot, cpRot, frameRot, bodyRot, curRot, curInFrameRot, desRot;
    Matrix JwBody, JwFrame, JtLoc, Je1, Je2, Je3;
    Vector e0, e0dot;

    /*!
     * Persistent pointers to parameters.  These pointers are maintained to prevent
     * having to call lookupParameter(...) every time getCommand(...) is called.
     */
    controlit::Parameter * paramActualPosition;
    controlit::Parameter * paramActualWorldPosition;
    controlit::Parameter * paramErrorQuat;

    /*!
     * Used to publish the goal and current 2D Orientation markers.
     */
    controlit::addons::ros::RealtimePublisher<visualization_msgs::MarkerArray>
        visualizationPublisher;

    /*!
     * The topic on which to publish the marker array messages for visualizing the
     * goal and current quaternions.
     */
    std::string markerTopic;

    double markerLength, markerWidth, markerHeight;
    Vector markerColorGoal, markerColorActual;  // alpha, red, green blue
    Vector markerOrientation; // for testing purposes

    // The following objects are used to publish marker messages.
    // geometry_msgs::Point startPoint;
    // geometry_msgs::Point endPoint;
    // visualization_msgs::Marker goalMarker;
    // visualization_msgs::Marker currMarker;
    Vector base_vector;
    Vector bodyToBaseTrans;

    Vector Q, Qd;
    Vector actualPositionTemp, actualWorldPositionTemp, errorQuatTemp;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace task_library
} // namespace controlit

#endif // __CONTROLIT_TASK_LIBRARY_ORIENT_QUATERNION_TASK_HPP__