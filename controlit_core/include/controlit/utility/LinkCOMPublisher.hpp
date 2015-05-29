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

#ifndef __CONTROLIT_LINK_COM_PUBLISHER__
#define __CONTROLIT_LINK_COM_PUBLISHER__

#include "ros/ros.h"

#include <controlit/ControlModel.hpp>

namespace controlit {
namespace utility {

/*!
 * Ramps up the torque from zero to the desired value
 * over a specified period of time.
 */
class LinkCOMPublisher
{
public:
  /*!
   * The constructor.
   */
  LinkCOMPublisher();

  /*!
   * Publishes the per-link COMs as a marker array.
   *
   * \param controlModel The control model from which the link COMs can be
   * obtained.
   * \return Whether this method executed sucessfully.
   */
  bool publishLinkCOMs(ControlModel & controlMode);

private:
  ros::NodeHandle nh;

  ros::Publisher publisher;
};

} // namespace utility
} // namespace controlit

#endif
