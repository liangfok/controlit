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

#ifndef __CONTROLIT_LOGGING_REAL_TIME_LOGGING_HPP__
#define __CONTROLIT_LOGGING_REAL_TIME_LOGGING_HPP__

#include <controlit/logging/Logging.hpp>

// The macros
// TODO: Not RT yet!! Fix!
#define CONTROLIT_DEBUG_RT CONTROLIT_DEBUG
#define CONTROLIT_INFO_RT  CONTROLIT_INFO
#define CONTROLIT_WARN_RT  CONTROLIT_WARN
#define CONTROLIT_ERROR_RT CONTROLIT_ERROR
#define CONTROLIT_FATAL_RT CONTROLIT_FATAL

#define CONTROLIT_DEBUG_COND_RT(cond) if (cond) CONTROLIT_DEBUG_RT
#define CONTROLIT_INFO_COND_RT(cond)  if (cond) CONTROLIT_INFO_RT
#define CONTROLIT_WARN_COND_RT(cond)  if (cond) CONTROLIT_WARN_RT
#define CONTROLIT_ERROR_COND_RT(cond) if (cond) CONTROLIT_ERROR_RT
#define CONTROLIT_FATAL_COND_RT(cond) if (cond) CONTROLIT_FATAL_RT

#endif