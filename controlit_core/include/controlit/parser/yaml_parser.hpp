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

#ifndef __CONTROLIT_YAML_PARSER_HPP__
#define __CONTROLIT_YAML_PARSER_HPP__

#include <controlit/addons/yaml/yaml_utilities.hpp>
#include <controlit/addons/yaml/yaml_math_utilities.hpp>

#include <controlit/BindingConfig.hpp>
#include <controlit/Parameter.hpp>
#include <controlit/ParameterReflection.hpp>

#include <controlit/parser/Header.hpp>
#include <controlit/parser/parse_list.hpp>

void parse_header(YAML::Node const& node, controlit::Header& hdr);
void emit_header(YAML::Emitter& out, controlit::Header const& hdr);

void parse_binding_config(YAML::Node const& node, controlit::BindingConfig& b);
void emit_binding_config(YAML::Emitter& out, controlit::BindingConfig const& b);

// HATE THIS!.. SOO much about parameters must change. Type
// information must be stored in YAML file.
template<class T>
void parse_parameter(const YAML::Node &in, controlit::Parameter* param)
{
	T val;
	in >> val;
	param->set(val);
}

void operator >> (YAML::Node const& node, controlit::Parameter* pr);

/*!
 * Parse a YAML iterator that contains information about a JointState message.
 *
 * The message's state should take the form:
 *
 * <pre>
 *   header:
 *     seq: 123456
 *     timestamp: 987.123 # in seconds
 *     frame_id: "boo"
 *   name: ["foo", "bar", "baz"]
 *   position: [1.1, 2.2, 3.3]
 *   velocity: [4.4, 5.5, 6.6]
 *   effort: [7.7, 8.8, 9.9]
 * </pre>
 *
 * \param[in] node The YAML node that contains the JointState message's state.
 * \param[out] param A pointer to the parameter that should hold the results.
 */
inline void parse_joint_state_parameter(const YAML::Node & node, controlit::Parameter * param)
{
	const YAML::Node & headerNode = node["header"];
	const YAML::Node & headerSeqNode = headerNode["seq"];
	const YAML::Node & headerTimeStampNode = headerNode["timestamp"];
	const YAML::Node & headerFrameIDNode = headerNode["frame_id"];

	const YAML::Node & nameNode = node["name"];
	const YAML::Node & positionNode = node["position"];
	const YAML::Node & velocityNode = node["velocity"];
	const YAML::Node & effortNode = node["effort"];

	// std::cout << __func__ << ": nameNode.size() = " << nameNode.size() << std::endl;

	// Instantiate and initialize a JointState message
	sensor_msgs::JointState js;

	js.name.clear();
	js.position.clear();
	js.velocity.clear();
	js.effort.clear();

	// Save the header
	headerSeqNode >> js.header.seq;

	double timeStamp;
	headerTimeStampNode >> timeStamp;
	js.header.stamp.sec = (int)timeStamp;
	js.header.stamp.nsec = (timeStamp - (int)timeStamp) * 1e9;

	headerFrameIDNode >> js.header.frame_id;

	// Ensure the dimensions of the joint state arrays are valid
	size_t numDOFs = nameNode.size();

	assert(positionNode.size() == numDOFs);
	assert(velocityNode.size() == numDOFs);
	assert(effortNode.size() == numDOFs);

	// Store the joint state into the JointState message
	for (size_t ii = 0; ii < numDOFs; ii++)
	{
		std::string name;
		double position, velocity, effort;

		nameNode[ii] >> name;
		positionNode[ii] >> position;
		velocityNode[ii] >> velocity;
		effortNode[ii] >> effort;

		js.name.push_back(name);
		js.position.push_back(position);
		js.velocity.push_back(velocity);
		js.effort.push_back(effort);

		// std::cout << __func__ << "Storing in JointState message: index = "
		//   << ii << ", name = " << name << ", position = " << position
		//   << ", velocity = " << velocity << ", effort = " << effort << std::endl;
	}

	param->set(js);
}

template<class T>
void serialize_parameter(YAML::Emitter& out, controlit::Parameter const& param)
{
	out << YAML::BeginMap
		<< YAML::Key 		<< "name"
		<< YAML::Value 		<< param.name()
		<< YAML::Key		<< "type"
		<< YAML::Value 		<< controlit::ParameterAccessor<T>::type(&param)
		<< YAML::Key 		<< "value"
		<< YAML::Value 		<< controlit::ParameterAccessor<T>::get(&param)
		<< YAML::EndMap;
}

YAML::Emitter& operator << (YAML::Emitter& out, controlit::Parameter const& v);

bool parse_parameter_reflection(YAML::Node const & node, controlit::ParameterReflection * pr);
void emit_parameter_reflection(YAML::Emitter & out, controlit::ParameterReflection const * v);

std::ostream& operator<<(std::ostream &os, YAML::Emitter const& out);


#endif // __CONTROLIT_YAML_PARSER_HPP__
