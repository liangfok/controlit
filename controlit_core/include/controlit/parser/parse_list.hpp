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

#ifndef _CONTROLIT_YAML_PARSER_PARSE_LIST_HPP_
#define _CONTROLIT_YAML_PARSER_PARSE_LIST_HPP_

// #include <drc/common/yaml_math_utilities.hpp>
#include <controlit/addons/yaml/yaml_utilities.hpp>
#include <vector>

namespace YAML
{
	void operator>>(const Node &in, std::vector<std::string> &x);
	Emitter &operator<<(Emitter &out, const std::vector<std::string> &x);
}

#endif
