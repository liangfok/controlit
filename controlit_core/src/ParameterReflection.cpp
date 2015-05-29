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

#include <controlit/ParameterReflection.hpp>
#include <controlit/ReflectionRegistry.hpp>

namespace controlit {

#define PRINT_INFO(ss)
// #define PRINT_INFO(ss) CONTROLIT_PR_INFO << ss;

ParameterReflection::ParameterReflection(std::string const& type_name,
    std::string const& instance_name) :
    typeName(type_name),
    instanceName(instance_name),
    useDefaultBindings(true)
{
    declareParameter("useDefaultBindings", &useDefaultBindings);
}

ParameterReflection::~ParameterReflection()
{
    for (ParameterMap::iterator ii(parameterMap.begin()); ii != parameterMap.end(); ++ii)
    {
        if (ii->second == nullptr)
        {
            CONTROLIT_PR_WARN << "Attempting to delete a null parameter!!!";
        }
        else
        {
            // CONTROLIT_PR_DEBUG << "Deleting parameter " << ii->second->name();
            delete ii->second;
        }
    }
}

BindingConfig * ParameterReflection::createROSInputBinding(std::string paramName, std::string dataType)
{
    BindingConfig * bc = new BindingConfig();
    bc->setParameter(paramName);
    bc->setDirection(controlit::BindingConfig::Direction::Input);
    bc->setTransportType("ROSTopic");
    bc->setTransportDataType(dataType);
    bc->addProperty("topic", instanceName + "/" + paramName);
    return bc;
}

BindingConfig * ParameterReflection::createROSOutputBinding(std::string paramName, std::string dataType, std::string latched, std::string publishRate, std::string queueSize)
{
    BindingConfig * bc = new BindingConfig();
    bc->setParameter(paramName);
    bc->setDirection(controlit::BindingConfig::Direction::Output);
    bc->setTransportType("ROSTopic");
    bc->setTransportDataType(dataType);
    bc->addProperty("latched", latched);
    bc->addProperty("publish_rate", publishRate);
    bc->addProperty("queue_size", queueSize);
    bc->addProperty("topic", instanceName + "/" + paramName);
    return bc;
}

        

bool ParameterReflection::hasParameter(std::string const& name) const
{
    auto const& tuple = parameterMap.find(name);
    return (tuple != parameterMap.end());
}

bool ParameterReflection::hasBinding(std::string const& name) const
{
    for (auto const& parameter : lookupParameters(controlit::PARAMETER_TYPE_BINDING))
    {
        if (parameter->getBindingConfig()->isForParameter(name))
            return true;
    }
    return false;
}


bool ParameterReflection::addEvent(std::string const& name, std::string const& expression)
{
    if (hasParameter(name))
    {
        CONTROLIT_ERROR << "Parameter " << name << " already exists in " << getInstanceName();
        return false;
    }
  
    Event event;
    event.name = name;
    event.condition.SetExpr(expression);
    event.condition.SetVarFactory(VariableFactory<ParameterReflection>, this);
    event.condition.DefineNameChars("0123456789_."
                         "abcdefghijklmnopqrstuvwxyz"
                         "ABCDEFGHIJKLMNOPQRSTUVWXYZ");
    event.enabled = true;
    events.push_back(event);
  
    // Record event as a new parameter
    addParameter(name, new double(0.0));
  
    PRINT_INFO("Added event '" << name << "', expression '" << expression << "'");
  
    return true;
}

// #if defined(__LP64__) || defined(_LP64)

// bool ParameterReflection::check(size_t const* param, size_t value) const
// {
//     return true;
// }

// #endif

// bool ParameterReflection::check(unsigned int const* param, unsigned int value) const
// {
//     return true;
// }

// bool ParameterReflection::check(int const* param, int value) const
// {
//     return true;
// }

// bool ParameterReflection::check(std::string const* param, std::string const& value) const
// {
//     return true;
// }

// bool ParameterReflection::check(double const* param, double value) const
// {
//     return true;
// }

// bool ParameterReflection::check(Vector const* param, Vector const& value) const
// {
//     return true;
// }

// bool ParameterReflection::check(Matrix const* param, Matrix const& value) const
// {
//     return true;
// }

// bool ParameterReflection::check(std::vector<std::string> const* param, std::vector<std::string> const& value) const
// {
//     return true;
// }

// bool ParameterReflection::check(BindingConfig const* param, BindingConfig const& value) const
// {
//     return true;
// }

ParameterReflection::ParameterList_t ParameterReflection::lookupParameters(ParameterType type) const
{
    ParameterList_t parameters;
    for (auto const& tuple : parameterMap)
    {
        if ((tuple.second)->isType(type))
        {
            parameters.push_back(tuple.second);
        }
    }
    return parameters;
}

Parameter* ParameterReflection::lookupParameter(std::string const& name)
{
    auto tuple = parameterMap.find(name);
    if (tuple == parameterMap.end())
    {
        CONTROLIT_PR_WARN << "Could not find parameter '" << name << "'";
        return NULL;
    }
    return tuple->second;
}

Parameter const* ParameterReflection::lookupParameter(std::string const& name) const
{
    auto const & tuple = parameterMap.find(name);
    if (tuple == parameterMap.end())
    {
        CONTROLIT_PR_WARN << "Could not find parameter '" << name << "'";
        return NULL;
    }
    return tuple->second;
}

Parameter* ParameterReflection::lookupParameter(std::string const& name, ParameterType type)
{
    Parameter* parameter = lookupParameter(name);
  
    if (parameter == NULL) return NULL;
    if (parameter->type() != type)
    {
        CONTROLIT_PR_WARN 
            << "Found parameter '" << name << "' but it was the wrong type. "
            << "Expected: " << type << ", actual: " << parameter->type();
        return NULL;
    }
    return parameter;
}

Parameter const* ParameterReflection::lookupParameter(std::string const& name, ParameterType type) const
{
    Parameter const* parameter = lookupParameter(name);
  
    if (parameter == NULL) return NULL;
    if (parameter->type() != type)
    {
        CONTROLIT_PR_WARN 
            << "Found parameter '" << name << "' but it was the wrong type."
            << " Expected: " << type << ", got: " << parameter->type();
        return NULL;
    }
    return parameter;
}

void ParameterReflection::dump(std::ostream& os, std::string const& title, std::string const& prefix) const
{
    if (!title.empty()) os << title << "\n";
  
    for (auto const& tuple : parameterMap)
        (tuple.second)->dump(os, prefix + "    ");
}

bool ParameterReflection::emitEvents()
{
    bool st = true;
  
    for (auto& event : events)
    {
        // Manually tie event evaluation to the parameter thats tracking it. Obvious thing
        // to do here is go back and refactor this code so that Events ARE Parameters..
        Parameter* param = lookupParameter(event.name);
        if (param == NULL)
        {
            CONTROLIT_PR_ERROR << "This should not have happened. Couldn't find event " << event.name << " in parameter list.";
            st = false;
            continue;
        }
    
        std::string eventName = getInstanceName() + ReflectionRegistry::ParameterNameDelimiter + event.name;
    
        // Evaluate condition and update parameter
        double val;
        try
        {
            val = event.condition.Eval();
        }
        catch (mu::Parser::exception_type const& e)
        {
            CONTROLIT_PR_ERROR 
                << "Expression eval failed: " << e.GetMsg() << "\n"
                << "-- Formula:  " << e.GetExpr() << "\n"
                << "-- Token:    " << e.GetToken() << "\n"
                << "-- Position: " << e.GetPos() << "\n"
                << "-- Errc:     " << e.GetCode();
            st = false;
            continue;
        }
    
        param->set(val);
        if (val > 0.5)
        {
            if (event.enabled) // event fire-once semantics
            {
                notifyListeners(eventName);
                PRINT_INFO("Firing event '" << event.name << "' (expression: " << event.condition.GetExpr() << " == true)");
                event.enabled = false; // event fire-once semantics
            }
            else
            {
                PRINT_INFO("Supressing event '" << event.name << "' (expression: " << event.condition.GetExpr() << " == true).");
            }
        }
        else
        {
            if (!event.enabled)
            {
                PRINT_INFO("Resetting event '" << event.name << "' (expression: " << event.condition.GetExpr() << " == false).");
                event.enabled = true;  // event fire-once semantics
            }
        }
    }
    return st;
}

bool ParameterReflection::getParameters(std::vector<std::string> & keys, std::vector<std::string> & values) const
{
    for (ParameterMap::const_iterator iter = parameterMap.begin(); iter != parameterMap.end(); ++iter)
    {
        Parameter * param = iter->second;
        keys.push_back(instanceName + "/" + iter->first);
        values.push_back(param->getValueString());
    }
    return true;
}

} // end namespace controlit
