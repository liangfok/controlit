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

#ifndef __CONTROLIT_LOGGING_LOGGING_HPP__
#define __CONTROLIT_LOGGING_LOGGING_HPP__

// look for ROS name, else generic name, else default name
#ifndef PACKAGE_NAME
#ifdef ROS_PACKAGE_NAME
#define PACKAGE_NAME ROS_PACKAGE_NAME
#else
#define PACKAGE_NAME "unknown_package"
#endif
#endif

#include "LogStream.hpp"

// The macros

#define CONTROLIT_LOG_INIT

#define CONTROLIT_LOG(pri) (std::ostream&)controlit::logging::LogStream<>(pri) \
    ("package", ROS_PACKAGE_NAME) \
    ("file", __FILE__) \
    ("function", __FUNCTION__) \
    ("pretty_function", __PRETTY_FUNCTION__) \
    ("line", std::to_string(__LINE__)) \
    ("pid", std::to_string(getpid()))

#define CONTROLIT_DEBUG CONTROLIT_LOG(controlit::logging::Priority::Debug)
#define CONTROLIT_INFO  CONTROLIT_LOG(controlit::logging::Priority::Info)
#define CONTROLIT_WARN  CONTROLIT_LOG(controlit::logging::Priority::Warning)
#define CONTROLIT_ERROR CONTROLIT_LOG(controlit::logging::Priority::Error)
#define CONTROLIT_FATAL CONTROLIT_LOG(controlit::logging::Priority::Fatal)

#define CONTROLIT_DEBUG_COND(cond) if (cond) CONTROLIT_DEBUG()
#define CONTROLIT_INFO_COND(cond)  if (cond) CONTROLIT_INFO()
#define CONTROLIT_WARN_COND(cond)  if (cond) CONTROLIT_WARN()
#define CONTROLIT_ERROR_COND(cond) if (cond) CONTROLIT_ERROR()
#define CONTROLIT_FATAL_COND(cond) if (cond) CONTROLIT_FATAL()

#endif
