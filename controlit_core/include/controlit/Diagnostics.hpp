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

#ifndef __CONTROLIT_CORE__DIAGNOSTICS_HPP__
#define __CONTROLIT_CORE__DIAGNOSTICS_HPP__

#include <vector>
// #include <chrono>
#include "ros/ros.h"

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include "controlit/DiagnosticsInfoProvider.hpp"

// message / service definitions
#include "controlit_core/get_parameters.h"

namespace controlit {

/*!
 * This is responsible for providing ControlIt! diagnostic information via
 * ROS service calls.  It also provides error and warning messages
 * via ROS topics.
 */
class Diagnostics
{
public:
  
    Diagnostics();
  
    /*!
     * The destructor.
     */
    ~Diagnostics() {}
  
    /*!
     * Initializes this class by advertising the various services and
     * topics it provides.
     *
     * \param[in] nh The ROS node handle with which to work with.
     * \param[in] infoProvider The object that provides the diagnostic
     * information.
     * \return Whether the initialization was successful.
     */
    bool init(ros::NodeHandle & nh, DiagnosticsInfoProvider * infoProvider);
  
    /*!
     * Publishes a message describing a WBC error.
     *
     * \param message A description of the error.
     * \return Whether the operation was successful.
     */
    bool publishError(std::string message);
  
    /*!
     * Publishes a message describing a WBC warning.
     *
     * \param message A description of the warning.
     * \return Whether the operation was successful.
     */
    bool publishWarning(std::string message);
  
private:
    /*!
     * The service handler for obtaining task parameters.
     */
    bool getTaskParametersHandler(
      controlit_core::get_parameters::Request  &req,
      controlit_core::get_parameters::Response &res);
  
    /*!
     * The service handler for obtaining constraint parameters.
     */
    bool getConstraintParametersHandler(
      controlit_core::get_parameters::Request  &req,
      controlit_core::get_parameters::Response &res);
  
    /*!
     * The service handler for obtaining the real joint indices
     */
    bool getRealJointIndicesHandler(
      controlit_core::get_parameters::Request  &req,
      controlit_core::get_parameters::Response &res);
  
    /*!
     * The service handler for obtaining the command joint indices
     */
    bool getCmdJointIndicesHandler(
      controlit_core::get_parameters::Request  &req,
      controlit_core::get_parameters::Response &res);
  
    /*!
     * The service handler for obtaining the first and most recent commands
     */
    bool getCommandsHandler(
      controlit_core::get_parameters::Request  &req,
      controlit_core::get_parameters::Response &res);
  
    /*!
     * Whether this object is initialized.
     */
    bool initialized;
  
    /*!
     * A pointer to the coordinator.
     */
    DiagnosticsInfoProvider * infoProvider;
  
    /*!
     * The publisher for WBC error messages.
     */
    ros::Publisher errorPublisher;
  
    /*!
     * The publisher for WBC warning messages.
     */
    ros::Publisher warningPublisher;
  
    /*!
     * The service that provides the compound task parameter values.
     */
    ros::ServiceServer taskParameterService;
  
    /*!
     * The service that provides the constraint set parameter values.
     */
    ros::ServiceServer constraintParameterService;
  
    /*!
     * The service that provides the indices of all real joints.
     */
    ros::ServiceServer realJointIndicesService;
  
    /*!
     * The service that provides the joint indices in the output command.
     */
    ros::ServiceServer cmdJointIndicesService;
  
    /*!
     * The service that provides the first and last commands issued.
     */
    ros::ServiceServer commandService;
  
    /*!
     * The error message to publish.
     */
    diagnostic_msgs::DiagnosticArray errorMessage;
  
    /*!
     * Whether an error was ever published.
     */
    bool publishedError;
  
    /*!
     * The warning message to publish.
     */
    diagnostic_msgs::DiagnosticArray warningMessage;
  
    /*!
     * Whether a warning was ever published.
     */
    bool publishedWarning;
};

} // namespace controlit

#endif
