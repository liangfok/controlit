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

#include <controlit/Diagnostics.hpp>
#include <controlit/Coordinator.hpp>
#include <controlit/utility/string_utility.hpp>
#include <controlit/logging/RealTimeLogging.hpp>

namespace controlit {

Diagnostics::Diagnostics()
  : initialized(false),
    infoProvider(nullptr),
    publishedError(false),
    publishedWarning(false)
{
}

bool Diagnostics::init(ros::NodeHandle & nh,
  DiagnosticsInfoProvider * infoProvider)
{
  if (initialized)
    return true;

  this->infoProvider = infoProvider;

  // Create the publishers
  errorPublisher   = nh.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics/errors",   1000);
  warningPublisher = nh.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics/warnings", 1000);

  taskParameterService = nh.advertiseService("diagnostics/getTaskParameters",
    &Diagnostics::getTaskParametersHandler, this);

  constraintParameterService = nh.advertiseService("diagnostics/getConstraintParameters",
    &Diagnostics::getConstraintParametersHandler, this);

  realJointIndicesService = nh.advertiseService("diagnostics/getRealJointIndices",
    &Diagnostics::getRealJointIndicesHandler, this);
  
  cmdJointIndicesService = nh.advertiseService("diagnostics/getCmdJointIndices",
    &Diagnostics::getCmdJointIndicesHandler, this);

  commandService = nh.advertiseService("diagnostics/getCommand",
    &Diagnostics::getCommandsHandler, this);

  // Create the error and warning messages
  diagnostic_msgs::DiagnosticStatus errorMsgContent;
  errorMsgContent.level = diagnostic_msgs::DiagnosticStatus::ERROR;
  errorMsgContent.name = nh.getNamespace();
  errorMsgContent.message = "[error description]";
  errorMsgContent.hardware_id = "n/a";

  this->errorMessage.status.push_back(errorMsgContent);

  diagnostic_msgs::DiagnosticStatus warningMsgContent;
  warningMsgContent.level = diagnostic_msgs::DiagnosticStatus::WARN;
  warningMsgContent.name = nh.getNamespace();
  warningMsgContent.message = "[warning description]";
  warningMsgContent.hardware_id = "n/a";

  this->warningMessage.status.push_back(warningMsgContent);

  // Reset some local state information
  publishedError = false;
  publishedWarning = false;
  initialized = true;

  return true;
}

bool Diagnostics::publishError(std::string message)
{
  // Compute the time since an error was last published
  // std::chrono::high_resolution_clock::time_point currTime = std::chrono::high_resolution_clock::now();
  // std::chrono::nanoseconds timeSpan = std::chrono::duration_cast<std::chrono::nanoseconds>(currTime - lastPublishErrorTime);
  // double period = timeSpan.count() / 1e9;

  // If an error has enver been published or the time since it was last published
  // exceeds the threshold period, publish the error message.
  // if (!publishedError || period > ERROR_MESSAGE_MIN_PUBLISH_PERIOD)
  // {

    diagnostic_msgs::DiagnosticStatus & errorMsgContent = errorMessage.status[0];

    errorMsgContent.message = message;

    errorMessage.header.stamp = ros::Time::now();

    errorPublisher.publish(errorMessage);

    // lastPublishErrorTime = std::chrono::high_resolution_clock::now();
    publishedError = true;
  // }
  // else
  // {
  //   CONTROLIT_WARN_RT << "Suppressing error message "
  //     << "(period " << period << " < " << ERROR_MESSAGE_MIN_PUBLISH_PERIOD << "): "
  //     << message;
  // }

  return true;
}

bool Diagnostics::publishWarning(std::string message)
{
  // Compute the time since a warning was last published
  // std::chrono::high_resolution_clock::time_point currTime = std::chrono::high_resolution_clock::now();
  // std::chrono::nanoseconds timeSpan = std::chrono::duration_cast<std::chrono::nanoseconds>(currTime - lastPublishWarningTime);
  // double period = timeSpan.count() / 1e9;

  // If a warning has enver been published or the time since it was last published
  // exceeds the threshold period, publish the error message.
  // if (!publishedWarning || period > WARNING_MESSAGE_MIN_PUBLISH_PERIOD)
  // {

    diagnostic_msgs::DiagnosticStatus & warningMsgContent = warningMessage.status[0];

    warningMsgContent.message = message;

    warningMessage.header.stamp = ros::Time::now();

    warningPublisher.publish(warningMessage);

    // lastPublishWarningTime = std::chrono::high_resolution_clock::now();
    publishedWarning = true;
  // }
  // else
  // {
  //   CONTROLIT_WARN_RT << "Suppressing warning message "
  //     << "(period " << period << " < " << WARNING_MESSAGE_MIN_PUBLISH_PERIOD << "): "
  //     << message;
  // }
  return true;
}

bool Diagnostics::getTaskParametersHandler(
  controlit_core::get_parameters::Request  &req,
  controlit_core::get_parameters::Response &res)
{
  std::vector<std::string> keys;
  std::vector<std::string> values;

  infoProvider->getTaskParameters(keys, values);

  assert(keys.size() == values.size());

  res.params.level = diagnostic_msgs::DiagnosticStatus::OK;
  res.params.name = "Task Parameters";
  res.params.message = "Here are the parameters of the tasks in this controller.";
  res.params.hardware_id = "N/A";

  diagnostic_msgs::KeyValue kv;

  for (size_t ii = 0; ii < keys.size(); ii++)
  {
    kv.key = keys[ii];
    kv.value = values[ii];

    res.params.values.push_back(kv);
  }

  return true;
}

bool Diagnostics::getConstraintParametersHandler(
  controlit_core::get_parameters::Request  &req,
  controlit_core::get_parameters::Response &res)
{

  std::vector<std::string> keys;
  std::vector<std::string> values;

  infoProvider->getConstraintParameters(keys, values);

  assert(keys.size() == values.size());

  res.params.level = diagnostic_msgs::DiagnosticStatus::OK;
  res.params.name = "Constraint Parameters";
  res.params.message = "Here are the parameters of the constraints in this controller.";
  res.params.hardware_id = "N/A";

  diagnostic_msgs::KeyValue kv;

  for (size_t ii = 0; ii < keys.size(); ii++)
  {
    kv.key = keys[ii];
    kv.value = values[ii];

    res.params.values.push_back(kv);
  }

  return true;
}

bool Diagnostics::getRealJointIndicesHandler(
  controlit_core::get_parameters::Request  &req,
  controlit_core::get_parameters::Response &res)
{

  const std::vector<std::string> & jointNames = infoProvider->getRealJointNames();  // return list of all joints

  res.params.level = diagnostic_msgs::DiagnosticStatus::OK;
  res.params.name = "Real Joint Indices";
  res.params.message = "Here are the joints and their indices.";
  res.params.hardware_id = "N/A";

  diagnostic_msgs::KeyValue kv;

  for (size_t ii = 0; ii < jointNames.size(); ii++)
  {
    kv.key = std::to_string(ii);
    kv.value = jointNames[ii];

    res.params.values.push_back(kv);
  }

  return true;
}

bool Diagnostics::getCmdJointIndicesHandler(
  controlit_core::get_parameters::Request  &req,
  controlit_core::get_parameters::Response &res)
{

  const std::vector<std::string> & jointNames = infoProvider->getActuatedJointNames();

  res.params.level = diagnostic_msgs::DiagnosticStatus::OK;
  res.params.name = "Command Joint Indices";
  res.params.message = "Here are the joints and their indices.";
  res.params.hardware_id = "N/A";

  diagnostic_msgs::KeyValue kv;

  for (size_t ii = 0; ii < jointNames.size(); ii++)
  {
    kv.key = std::to_string(ii);
    kv.value = jointNames[ii];

    res.params.values.push_back(kv);
  }

  return true;
}

bool Diagnostics::getCommandsHandler(
  controlit_core::get_parameters::Request  &req,
  controlit_core::get_parameters::Response &res)
{
  // Initialize the result message's header
  res.params.level = diagnostic_msgs::DiagnosticStatus::OK;
  res.params.name = "Commands";
  res.params.message = "Here are the first and most recent commands issued";
  res.params.hardware_id = "N/A";

  diagnostic_msgs::KeyValue kv;

  // Save the first and last commands
  // kv.key = "First Command";
  // kv.value = infoProvider->getFirstCommand()->toString();
  // res.params.values.push_back(kv);

  kv.key = "Last Command";
  kv.value = infoProvider->getLastCommand()->toString();
  res.params.values.push_back(kv);

  return true;
}

} // namespace controlit
