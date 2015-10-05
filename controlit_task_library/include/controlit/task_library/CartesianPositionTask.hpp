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

#ifndef __CONTROLIT_TASK_LIBRARY_CARTESIAN_POSITION_TASK__
#define __CONTROLIT_TASK_LIBRARY_CARTESIAN_POSITION_TASK__

#include <controlit/LatchedTask.hpp>
#include <controlit/task_library/PDController.hpp>
#include <controlit/addons/eigen/LinearAlgebra.hpp>

namespace controlit {
namespace task_library {

using controlit::addons::eigen::Vector;
using controlit::addons::eigen::Matrix;
using controlit::addons::eigen::Vector3d;
using controlit::addons::eigen::Matrix3d;

/*!
 * This is a cartesian (operation space) position task.
 * It moves an end effector to a goal position.
 *
 * \note This task is always three dimensional and it relies on the
 * SaturationPolicy::NormVel policy, so the gains and maxVelocity have to
 * be one-dimensional.
 *
 * Parameters (see also PDTask for inherited parameters):
 * - end_effector (string): name of the end effector link
 * - controlPoint (vector): reference point wrt end effector frame
 */
class CartesianPositionTask : public controlit::LatchedTask
{
public:
    /*!
     * The default constructor.
     */
    CartesianPositionTask();

    /*!
     * Initializes this task.
     *
     * \param[in] model The control model used to execute this task.
     */
    virtual bool init(ControlModel & model);

    /*!
     * Computes the current Cartesian position.
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
     * The name of the body on the robot that defines the coordinate frame
     * of the control point--can be a joint name or a link name
     */
    std::string bodyName_;

    /*!
     * The ID of the body on the robot that defines the coordinate frame
     * of the control point.
     */
    unsigned int bodyId_;

    /*!
     * This is the cartesian point on the robot that is being controlled.
     * It is defined in terms of the body specified by bodyId_ and bodyName_.
     */
    Vector controlPoint_;

    /*!
     * The goal position in the frameName_ frame
     */
    Vector goalPosition_;

    /*!
     * The goal velocity in the frameName_ frame
     */
    Vector goalVelocity_;

    /*!
     * The PROJECTED actual position in the frameName_ frame
     */
    Vector actualPosition_;

    /*!
     * The PROJECTED actual position in the world frame
     */
    Vector actualWorldPosition_;

    /*!
     * The PROJECTED actual velocity in the frameName_ frame
     */
    Vector actualVelocity_;

    /*!
     * The PROJECTED actual velocity in the world frame
     */
    Vector actualWorldVelocity_;

    /*!
     * Projection matrix in the frameName_ frame to allow subspace control
     */
    Matrix projection_;

    /*!
     * A PD controller
     */
    std::unique_ptr<PDController> controller;

private:

    void addDefaultBindings();

    /*!
     * The Jacobian of the task's reference frame and other utilities
     */
    Matrix JvBody;
    Matrix JwBody;
    Matrix JvFrame;
    Matrix JwFrame;
    Matrix JtLoc;

    Vector xCurWorld;
    Vector xCurFrameProjected;
    Vector xCurWorldProjected;

    Vector xDesFrameProjected;
    Vector xDesWorldProjected;

    // Variables used by sense()
    Vector Q, Qd;
    Vector3d bodyTranslation;
    Matrix3d bodyRotation;
    Matrix3d frameRotation;
    Vector3d frameTranslation;
    Vector3d ePos, eVel;

    /*!
     * Persistent pointers to parameters.  These pointers are maintained to prevent
     * having to call lookupParameter(...) every time getCommand(...) is called.
     */
    controlit::Parameter * paramActualPosition;
    controlit::Parameter * paramActualWorldPosition;
    controlit::Parameter * paramActualVelocity;
    controlit::Parameter * paramActualWorldVelocity;
};

} // namespace task_library
} // namespace controlit

#endif // __CONTROLIT_TASK_LIBRARY_CARTESIAN_POSITION_TASK__
