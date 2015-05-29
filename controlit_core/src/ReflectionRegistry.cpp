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

#include <controlit/ReflectionRegistry.hpp>
#include <controlit/logging/RealTimeLogging.hpp>
#include <controlit/addons/cpp/StringUtilities.hpp>
#include <controlit/utility/string_utility.hpp>

namespace controlit {

ReflectionRegistry::ReflectionRegistry(std::string const& type_name,
  std::string const& instance_name) :
  ParameterReflection(type_name, instance_name)
{

}

bool ReflectionRegistry::addParameterCollection(std::shared_ptr<ParameterReflection> unmanagedCollection)
{
  auto const& tuple = parameterCollections.find(unmanagedCollection->getInstanceName());
  if (tuple == parameterCollections.end())
  {
    parameterCollections.insert(
      std::make_pair(unmanagedCollection->getInstanceName(), unmanagedCollection) );

    // CONTROLIT_PR_INFO << "Added parameter collection '" << unmanagedCollection->getInstanceName() << "', type: "
    //             << unmanagedCollection->getTypeName() << ")";

    return true;
  }

  CONTROLIT_PR_WARN << "Request to add '" << unmanagedCollection->getInstanceName() << "', type: "
              << unmanagedCollection->getTypeName() << ")," << " but already have '"
              << (tuple->second)->getInstanceName() << "', type: " << (tuple->second)->getTypeName() << ")"
              << ". Ignoring request.";

  return false;
}

void ReflectionRegistry::clearParameterCollection()
{
  parameterCollections.clear();
}

std::weak_ptr<ParameterReflection> ReflectionRegistry::getParameterCollection(std::string const& name)
{
  std::weak_ptr<ParameterReflection> ptr;

  auto tuple = parameterCollections.find(name);
  if (tuple != parameterCollections.end())
  {
    ptr = tuple->second;
    return ptr;
  }

  CONTROLIT_PR_WARN << "Could not find requested collection '" << name << "'";
  return ptr;
}

bool ReflectionRegistry::addEvent(std::string const& name, std::string const& expression)
{
  if (hasParameter(name))
  {
    CONTROLIT_PR_WARN << "Already have an event named '" << name << "'. Ignoring request.";
    return false;
  }

  // CONTROLIT_PR_INFO << "Adding event (" << name << "), Expression: \"" << expression << "\"";

  Event event;
  event.name = name;
  event.condition.SetExpr(expression);
  event.condition.SetVarFactory(VariableFactory<ReflectionRegistry>, this);
  event.condition.DefineNameChars("0123456789_."
                       "abcdefghijklmnopqrstuvwxyz"
                       "ABCDEFGHIJKLMNOPQRSTUVWXYZ");

  events.push_back(event);

  // Record event as a new parameter
  addParameter(name, new double);

  // CONTROLIT_PR_INFO << "Added event '" << name << "', expression '" << expression << "'";

  return true;
}

bool ReflectionRegistry::hasParameter(std::string const& name)
{
  return (getParameter(name) != NULL);
}

Parameter* ReflectionRegistry::lookupParameter(std::string const& qualifiedParameterName)
{
  Parameter* p = getParameter(qualifiedParameterName);
  if (p == NULL)
  {
    controlit::addons::cpp::string_splitter splitter(qualifiedParameterName, ParameterNameDelimiter);
    std::string registryName = splitter.next();
    CONTROLIT_PR_WARN << "Could not find parameter collection '" << registryName << "'";
  }

  return p;
}

Parameter* ReflectionRegistry::getParameter(std::string const& qualifiedParameterName)
{
  controlit::addons::cpp::string_splitter splitter(qualifiedParameterName, ParameterNameDelimiter);
  std::string registryName = splitter.next();

  // No '.' found.. parameter requested belongs to this ParameterReflection
  if (splitter.rest() == "") return ParameterReflection::lookupParameter(qualifiedParameterName);

  auto tuple = parameterCollections.find(registryName);
  if (tuple != parameterCollections.end())
    return (tuple->second)->lookupParameter(splitter.rest());
  else
    return NULL;
}

Parameter const* ReflectionRegistry::lookupParameter(std::string const& qualifiedParameterName) const
{
  controlit::addons::cpp::string_splitter splitter(qualifiedParameterName, ParameterNameDelimiter);
  std::string registryName = splitter.next();

  // No '.' found.. parameter requested belongs to this ParameterReflection
  if (splitter.rest() == "") return ParameterReflection::lookupParameter(qualifiedParameterName);

  auto const& tuple = parameterCollections.find(registryName);
  if (tuple != parameterCollections.end())
  {
    return (tuple->second)->lookupParameter(splitter.rest());
  }

  CONTROLIT_PR_WARN << "Could not find parameter collection '" << registryName << "'";

  return NULL;
}

Parameter* ReflectionRegistry::lookupParameter(std::string const& qualifiedParameterName, ParameterType type)
{
  Parameter* parameter = lookupParameter(qualifiedParameterName);

  // Not an elegant way to do this.. but this is all that is happening at the ParameterReflection level as well..
  if (parameter == NULL) return NULL;
  if (!parameter->isType(type))
  {
    CONTROLIT_PR_WARN << "Found parameter '" << qualifiedParameterName << "' but it was the wrong type. "
                << "Expected: " << type << ", actual: " << parameter->type();
    return NULL;
  }

  return parameter;
}

Parameter const* ReflectionRegistry::lookupParameter(std::string const& qualifiedParameterName, ParameterType type) const
{
  Parameter const* parameter = lookupParameter(qualifiedParameterName);

  // Not an elegant way to do this.. but this is all that is happening at the ParameterReflection level as well..
  if (parameter == NULL) return NULL;
  if (!parameter->isType(type))
  {
    CONTROLIT_PR_WARN << "Found parameter '" << qualifiedParameterName << "' but it was the wrong type. "
                << "Expected: " << type << ", actual: " << parameter->type();
    return NULL;
  }

  return parameter;
}

bool ReflectionRegistry::getParameters(std::vector<std::string> & keys, std::vector<std::string> & values) const
{
  // Call superclass' getParameters() method
  if (!ParameterReflection::getParameters(keys, values)) return false;

  // Get parameters in each collection
  for (ParameterCollectionMap_t::const_iterator iter = parameterCollections.begin(); iter != parameterCollections.end(); ++iter)
  {
    if (!iter->second->getParameters(keys, values)) return false;
  }

  return true;
}

void ReflectionRegistry::dump(std::ostream& os, std::string const& title, std::string const& prefix) const
{
    ParameterReflection::dump(os, title, prefix);
  
    for (auto const& tuple : parameterCollections)
        (tuple.second)->dump(os, title, prefix + "    ");
}

bool ReflectionRegistry::emitEvents()
{
    for (auto& tuple : parameterCollections)
    {
        if (!(tuple.second)->emitEvents()) return false;
    }
    return ParameterReflection::emitEvents();
}

} // end namespace controlit
