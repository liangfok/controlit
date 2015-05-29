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

#ifndef __CONTROLIT_CORE_YAML_FACTORY_HPP__
#define __CONTROLIT_CORE_YAML_FACTORY_HPP__

#include <map>
#include <controlit/parser/Header.hpp>
#include <controlit/parser/yaml_parser.hpp>

using controlit::addons::yaml::yaml_read_file;
using controlit::addons::yaml::yaml_read_string;

namespace controlit {

// Curiously recurring template pattern
template<class Derived, class CreatedType>
class YamlFactory
{
protected:
    YamlFactory() {};

public:
    CreatedType* loadFromFile(boost::filesystem::path const& filePath)
    {
        YAML::Node doc;
        yaml_read_file(filePath, doc);
        Header hdr = parseHeader(doc);  // Read header
        return loadFromYaml(doc);
    }
  
    CreatedType* loadFromString(std::string const& yamlString)
    {
        YAML::Node doc;
        yaml_read_string(yamlString, doc);
        Header hdr = parseHeader(doc);  // Read header
        return loadFromYaml(doc);
    }
  
    CreatedType* loadFromYaml(YAML::Node const& node)
    {
        return static_cast<Derived*>(this)->loadFromYamlImpl(node);
    }
  
private:

    Header parseHeader(YAML::Node const& doc) const
    {
        YAML::Node const* pHeaderNode = doc.FindValue("header");
        if (pHeaderNode == NULL)
        {
            throw std::runtime_error("Header section required.");
        }
    
        Header hdr;
        parse_header(*pHeaderNode, hdr);
        return hdr;
    }
};

}

#endif
