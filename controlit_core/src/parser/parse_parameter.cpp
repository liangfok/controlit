#include <controlit/parser/yaml_parser.hpp>

YAML::Emitter& operator << (YAML::Emitter& out, controlit::Parameter const& rhs)
{
	switch (rhs.type())
	{
		case controlit::PARAMETER_TYPE_STRING: 	serialize_parameter<std::string>(out, rhs);		break;
		case controlit::PARAMETER_TYPE_INTEGER:	serialize_parameter<int>(out, rhs);				break;
		case controlit::PARAMETER_TYPE_REAL:		serialize_parameter<double>(out, rhs);			break;
		case controlit::PARAMETER_TYPE_VECTOR:	serialize_parameter<Vector>(out, rhs);		break;
		case controlit::PARAMETER_TYPE_MATRIX:	serialize_parameter<Matrix>(out, rhs);		break;
		case controlit::PARAMETER_TYPE_LIST:		serialize_parameter< std::vector<std::string> >(out, rhs);		break;
		case controlit::PARAMETER_TYPE_JOINT_STATE: serialize_parameter<sensor_msgs::JointState>(out, rhs); break;
		default:							out << YAML::Null;								break;
	}

	return out;
}