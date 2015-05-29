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

#include <pluginlib/class_list_macros.h>
#include <controlit/Constraint.hpp>
#include <controlit/constraint_library/ConstraintLibrary.hpp>

// Defined in /opt/ros/groovy/include/pluginlib/class_list_macros.h:
//
// PLUGINLIB_EXPORT_CLASS(class_type, base_class_type)

PLUGINLIB_EXPORT_CLASS(controlit::constraint_library::FlatContactConstraint,       controlit::Constraint);
PLUGINLIB_EXPORT_CLASS(controlit::constraint_library::PointContactConstraint,      controlit::Constraint);
PLUGINLIB_EXPORT_CLASS(controlit::constraint_library::TransmissionConstraint,      controlit::Constraint);
PLUGINLIB_EXPORT_CLASS(controlit::constraint_library::FlatContactConstraintSensed, controlit::Constraint);
PLUGINLIB_EXPORT_CLASS(controlit::constraint_library::OmniWheelContactConstraint,  controlit::Constraint);
