#include <controlit/parser/yaml_parser.hpp>

void parse_header(YAML::Node const & node, controlit::Header & hdr)
{
    // node["version"] >> hdr.version;
    YAML::Node const* pNode = node.FindValue("description");
    if (pNode != NULL)
    {
        (*pNode) >> hdr.description;
    }
}

void emit_header(YAML::Emitter& out, controlit::Header const& hdr)
{
    out << YAML::BeginMap
        // << YAML::Key         << "version"
        // << YAML::Value       << hdr.version
        << YAML::Key        << "description"
        << YAML::Value      << hdr.description
        << YAML::EndMap;
}