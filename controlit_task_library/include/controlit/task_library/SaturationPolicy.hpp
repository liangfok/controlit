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

#ifndef __CONTROLIT_TASK_LIBRARY_SATURATION_POLICY_HPP_
#define __CONTROLIT_TASK_LIBRARY_SATURATION_POLICY_HPP_

namespace controlit {
namespace task_library {

// I really don't like this outside, in the task_library
// namespace.. but oh well, it makes the SMController.cpp
// code prettier.
struct SaturationPolicy
{
  enum Options : int
  {
    /*!
     * Do not saturate commands (use with caution).
     */
    Off = 0,
    /*!
     * Each component of the position-error is scaled according to an individual
     * saturation term
     */
    ComponentWiseVel,
    /*!
     * Similar to SATURATION_COMPONENT_WISE but the most saturated component
     * determines the overall scaling.
     */
    MaxComponentVel,
    /*!
     * The vector norm of the position error is
     * used to determine the saturation factor (this only makes
     * sense if kp, kd, and maxVelocity are one-dimensional values so
     * this constraint is enforced in other parts of the SMTask
     * implementation as well)
     */
    NormVel,

     /*!
     * Each component of the position-error is scaled according to an individual
     * saturation term
     */
    ComponentWiseAcc,
    /*!
     * Similar to SATURATION_COMPONENT_WISE but the most saturated component
     * determines the overall scaling.
     */
    MaxComponentAcc,
    /*!
     * The vector norm of the position error is
     * used to determine the saturation factor (this only makes
     * sense if kp, kd, and maxVelocity are one-dimensional values so
     * this constraint is enforced in other parts of the SMTask
     * implementation as well)
     */
    NormAcc
  };

static std::string SaturationPolicyToString(int sp)
{
    switch(sp)
    {
        case SaturationPolicy::Off: return "Off";
        case SaturationPolicy::ComponentWiseVel: return "ComponentWiseVel";
        case SaturationPolicy::ComponentWiseAcc: return "ComponentWiseAcc";
        case SaturationPolicy::MaxComponentVel: return "MaxComponentVel";
        case SaturationPolicy::NormVel: return "NormVel";
        case SaturationPolicy::MaxComponentAcc: return "MaxComponentAcc";
        case SaturationPolicy::NormAcc: return "NormAcc";
        default: return "Unknown";
    }

    return "Unknown";
}
};

} // namespace task_library
} // namespace controlit

#endif
