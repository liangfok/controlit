#include <controlit/parser/yaml_parser.hpp>

#include <controlit/ParameterReflection.hpp>

bool parse_parameter_reflection(YAML::Node const & node, controlit::ParameterReflection * pr)
{
    // CONTROLIT_INFO("controlit_instance_name", pr->getInstanceName())("controlit_type", pr->getTypeName())
    //     << "Method Called! Parsing YAML file for parameters, events, and bindings...";
  
    // Parse parameters
    if (YAML::Node const* paramsNode = node.FindValue("parameters"))
    {
        if (paramsNode == nullptr)
        {
            CONTROLIT_ERROR("controlit_instance_name", pr->getInstanceName())("controlit_type", pr->getTypeName())
                << "Failed to find 'parameters' section in YAML file!";
            return false;
        }
    
        for(YAML::Iterator it = paramsNode->begin(); it != paramsNode->end(); ++it)
        {
            std::string paramName;
            (*it)["name"] >> paramName;
      
            controlit::Parameter * parameter = pr->lookupParameter(paramName);
      
            // CONTROLIT_INFO("controlit_instance_name", pr->getInstanceName())("controlit_type", pr->getTypeName())
            //     << "Parsing parameter '" << paramName;
      
            if (parameter != nullptr)
            {
                switch (parameter->type())
                {
                    case controlit::PARAMETER_TYPE_STRING:  parse_parameter<std::string>((*it)["value"], parameter);  break;
                    case controlit::PARAMETER_TYPE_INTEGER: parse_parameter<int>((*it)["value"], parameter);          break;
                    case controlit::PARAMETER_TYPE_REAL:    parse_parameter<double>((*it)["value"], parameter);       break;
                    case controlit::PARAMETER_TYPE_VECTOR:  parse_parameter<Vector>((*it)["value"], parameter);  break;
                    case controlit::PARAMETER_TYPE_MATRIX:  parse_parameter<Matrix>((*it)["value"], parameter);  break;
                    case controlit::PARAMETER_TYPE_LIST:    parse_parameter< std::vector<std::string> >((*it)["value"], parameter);  break;
                    case controlit::PARAMETER_TYPE_JOINT_STATE: parse_joint_state_parameter((*it)["values"], parameter); break;
                    case controlit::PARAMETER_TYPE_VOID:
                    case controlit::PARAMETER_TYPE_BINDING:
                    default:
                    break;
                }
            }
            else
            {
                CONTROLIT_WARN("controlit_instance_name", pr->getInstanceName())("controlit_type", pr->getTypeName())
                    << "Ignoring parameter '" << paramName << "' because it is NULL!";
            }
        }
    }
  
    // Parse events
    if (YAML::Node const* eventsNode = node.FindValue("events"))
    {
        for(YAML::Iterator it = eventsNode->begin(); it != eventsNode->end(); ++it)
        {
            std::string eventName, expression;
            (*it)["name"] >> eventName;
            (*it)["expression"] >> expression;
            if (!pr->addEvent(eventName, expression)) return false;
        }
    }
  
    // Parse bindings
    if (YAML::Node const* bindingNodes = node.FindValue("bindings"))
    {
        for(YAML::Iterator it = bindingNodes->begin(); it != bindingNodes->end(); ++it)
        {
            controlit::BindingConfig *pBindingConfig = new controlit::BindingConfig;
            parse_binding_config((*it), *pBindingConfig);
            pr->addParameter(pBindingConfig->name(), pBindingConfig);
        }
    }

    return true;
}

void emit_event(YAML::Emitter& out, controlit::Event const& event)
{
    out << YAML::BeginMap
        << YAML::Key     << "name"
        << YAML::Value   << event.name
        << YAML::Key     << "expression"
        << YAML::Value   << event.condition.GetExpr()
        << YAML::EndMap;
}

void emit_parameter_reflection(YAML::Emitter& out, controlit::ParameterReflection const* pr)
{
    out << YAML::BeginMap
        << YAML::Key        << "type"
        << YAML::Value      << pr->getTypeName()
        << YAML::Key        << "name"
        << YAML::Value      << pr->getInstanceName()
        << YAML::Key        << "parameters"
        << YAML::Value      << YAML::BeginSeq;
  
    for (auto const& tuple : pr->getParameterTable())
    {
        out << *(tuple.second);
    }
  
    out << YAML::EndSeq
        << YAML::Key      << "events"
        << YAML::Value    << YAML::BeginSeq;
  
    for (auto const& event : pr->getEvents())
    {
        emit_event(out, event);
    }
    out << YAML::EndSeq;
  
    out << YAML::Key      << "bindings"
        << YAML::Value    << YAML::BeginSeq;
  
    for (auto const& parameter : pr->lookupParameters(controlit::PARAMETER_TYPE_BINDING))
    {
        emit_binding_config(out, *(parameter->getBindingConfig()));
    }
  
    out << YAML::EndSeq
        << YAML::EndMap;
}
