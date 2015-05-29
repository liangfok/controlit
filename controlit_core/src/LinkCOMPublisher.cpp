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
#include <controlit/utility/LinkCOMPublisher.hpp>
#include <visualization_msgs/MarkerArray.h>

namespace controlit {
namespace utility {

LinkCOMPublisher::LinkCOMPublisher()
{
  publisher = nh.advertise<visualization_msgs::MarkerArray>("link_COMs_marker_array", 0);
}

bool LinkCOMPublisher::publishLinkCOMs(ControlModel & controlModel)
{
  // Obtain the COMs of each link.
  Matrix linkCOMs;
  if (!controlModel.getLinkCOMs(linkCOMs)) return false;

  // Define a MarkerArray message
  visualization_msgs::MarkerArray markerArray;
  markerArray.markers.resize(0);

  for (int ii = 0; ii < linkCOMs.cols(); ii++)
  {
    std::stringstream nameSpaceBuff;
    nameSpaceBuff << "LinkCOM_" << ii;

    // Define marker 1, which is the actual position in the world coordinate frame.
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/testbed_base";
    marker.header.stamp = ros::Time();
    marker.ns = nameSpaceBuff.str();
    marker.id = ii;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = linkCOMs(0, ii);
    marker.pose.position.y = linkCOMs(1, ii);
    marker.pose.position.z = linkCOMs(2, ii);
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;  // 5cm x 5cm x 5cm pink ball
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    markerArray.markers.push_back(marker);
  }

  publisher.publish(markerArray);

  return true;
}

} // namespace utility
} // namespace controlit
