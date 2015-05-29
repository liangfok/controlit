#include <controlit/parser/yaml_parser.hpp>

//void operator >> (YAML::Node const& node, controlit::BindingConfig& b)
// namespace v1
// {
//     void parse_binding_config(YAML::Node const& node, controlit::BindingConfig & bc)
//     {
//         // node["transportType"] >> bc.transportType;
//         std::string transportType;
//         std::string transportDataType;

//         node["transportType"] >> transportType;
//         node["transportDataType"] >> transportDataType;

//         bc.setTransportType(transportType);
//         bc.setTransportDataType(transportDataType);

//         std::string topic;
//         node["topic"] >> topic;
//         bc.addProperty("topic", topic);

//         std::string parameter;
//         node["parameter"] >> 
//         for (YAML::Iterator it = node["parameters"].begin();
//              it !=node["parameters"].end(); ++it)
//         {
//             std::string paramName;
//             *it >> paramName;
//             bc.addParameter(paramName);
//         }

//         std::string direction;
//         node["direction"] >> direction;

//         bc.setDirection(controlit::BindingConfig::Direction::Undefined);
//         if (direction == "input") bc.setDirection(controlit::BindingConfig::Direction::Input);
//         if (direction == "output") bc.setDirection(controlit::BindingConfig::Direction::Output);
//         if (direction == "bidirectional") bc.setDirection(controlit::BindingConfig::Direction::Bidirectional);
//     }

//     void emit_binding_config(YAML::Emitter& out, controlit::BindingConfig const & bc)
//     {
//         out << YAML::BeginMap
//             << YAML::Key    << "transportType"
//             << YAML::Value  << bc.getTransportType();

//         out << YAML::Key << "transportDataType"
//             << YAML::Value << bc.getTransportDataType();

//         if (bc.hasProperty("topic"))
//         {
//             out << YAML::Key    << "topic"
//                 << YAML::Value  << bc.getProperty("topic");
//         }

//         out << YAML::Key    << "direction"
//             << YAML::Value;

//         switch (bc.getDirection())
//         {
//             case controlit::BindingConfig::Direction::Undefined:        out << YAML::Null;      break;
//             case controlit::BindingConfig::Direction::Input:            out << "input";         break;
//             case controlit::BindingConfig::Direction::Output:           out << "output";        break;
//             case controlit::BindingConfig::Direction::Bidirectional:    out << "bidirectional"; break;
//         }

//         out << YAML::Key    << "parameters"
//             << YAML::Value  << YAML::Flow   << bc.getParameters()
//             << YAML::EndMap;
//     }
// }

/*!
 * Parses the binding config YAML specification.
 *
 * \param[in] node The YAML node containing the binding config specification.
 * \param[out] b The object in which to store the binding configuration information.
 */
void parse_binding_config(YAML::Node const& node, controlit::BindingConfig& bc)
{
    // Parameters
    // for (YAML::Iterator it = node["parameters"].begin();
    //      it !=node["parameters"].end(); ++it)
    // {
    //     std::string paramName;
    //     *it >> paramName;
    //     bc.addParameter(paramName);
    // }

    std::string parameter;
    node["parameter"] >> parameter;
    bc.setParameter(parameter);

    // Direction
    std::string direction;
    node["direction"] >> direction;

    bc.setDirection(controlit::BindingConfig::Direction::Undefined);
    if (direction == "input") bc.setDirection(controlit::BindingConfig::Direction::Input);
    if (direction == "output") bc.setDirection(controlit::BindingConfig::Direction::Output);
    if (direction == "bidirectional") bc.setDirection(controlit::BindingConfig::Direction::Bidirectional);

    // Target
    YAML::Node const& targetNode = *(node.FindValue("target"));
    // targetNode["transportType"] >> bc.transportType;
    std::string transportType;
    std::string transportDataType;

    targetNode["type"] >> transportType;
    targetNode["dataType"] >> transportDataType;

    bc.setTransportType(transportType);
    bc.setTransportDataType(transportDataType);

    // Target properties
    YAML::Node const* propertiesNode = targetNode.FindValue("properties");
    if (propertiesNode != NULL)
    {
        for (YAML::Iterator it = propertiesNode->begin(); it != propertiesNode->end(); ++it)
        {
            std::string key, value;
            it.first() >> key;
            it.second() >> value;
            bc.addProperty(key, value);
        }
    }
}

void emit_binding_config(YAML::Emitter& out, controlit::BindingConfig const& bc)
{
    out << YAML::BeginMap
        << YAML::Key    << "parameter"
        << YAML::Value  << bc.getParameter()
        << YAML::Key    << "direction"
        << YAML::Value;

    switch (bc.getDirection())
    {
        case controlit::BindingConfig::Direction::Undefined:        out << YAML::Null;      break;
        case controlit::BindingConfig::Direction::Input:            out << "input";         break;
        case controlit::BindingConfig::Direction::Output:           out << "output";        break;
        case controlit::BindingConfig::Direction::Bidirectional:    out << "bidirectional"; break;
    }

    out << YAML::Key    << "target"
        << YAML::Value  << YAML::BeginMap
        << YAML::Key    << "transportType"
        << YAML::Value  << bc.getTransportType()
        << YAML::Key    << "transportDataType"
        << YAML::Value  << bc.getTransportDataType()
        << YAML::Key    << "properties"
        << YAML::Value  << YAML::BeginMap;

    for (auto const& tuple : bc.getProperties())
    {
        out << YAML::Key    <<  tuple.first
            << YAML::Value  <<  tuple.second;
    }

    out << YAML::EndMap     // End properties map
        << YAML::EndMap     // End target map
        << YAML::EndMap;    // End binding_config map
}

// void parse_binding_config(YAML::Node const& node, controlit::BindingConfig& bc)
// {
//     parse_binding_config(node, bc);
// }

// void emit_binding_config(YAML::Emitter& out, controlit::BindingConfig const& bc)
// {
//     emit_binding_config(out, bc);
// }
