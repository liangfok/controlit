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
 
#ifndef __CONTROLIT_DEPENDENCY_ADDONS_UTILITIES__
#define __CONTROLIT_DEPENDENCY_ADDONS_UTILITIES__

#include <ostream>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>

namespace controlit {
namespace addons {
namespace yaml {

// Discard alias?
namespace bfile = boost::filesystem;

void yaml_read_file(const boost::filesystem::path &filePath, YAML::Node &doc);
void yaml_write_file(const boost::filesystem::path &filePath, const YAML::Emitter &out);

void yaml_read_string(const std::string &str, YAML::Node &doc);
void yaml_write_string(std::string &str, YAML::Emitter const &out);


/**
 * \brief Read one line from std::cin.
 */
void yaml_read_stdin(YAML::Node &doc, bool ignoreFirst = true);

template<typename T>
void yaml_read_string(const std::string &str, T &var)
{
    YAML::Node doc;
    yaml_read_string(str, doc);
    doc >> var;
}

template<typename K, typename T>
void yaml_read_default(const YAML::Node &doc, const K &key, T &value, const T &def = T())
{
    const YAML::Node *pNode = doc.FindValue(key);
    if (pNode)
        (*pNode) >> value;
    else
        value = def;
}

template<typename K, typename T>
T yaml_get_default(const YAML::Node &doc, const K &key, const T &def = T())
{
    T value;
    yaml_read_default(doc, key, value, def);
    return value;
}

/**
 * \brief Try reading value from a file specified as a string (relative to given path), otherwise read the value directly from the entry
 * \note Not good for sent files
 */
template<typename T>
void yaml_read_or_load(const YAML::Node &node, T &value, boost::filesystem::path relPath = "", const std::string &loadName = "")
{
    std::string nameTest;
    if (node.Read(nameTest))
    {
        // It's a file
        YAML::Node newNode;
        yaml_read_file(relPath / (nameTest + ".yml"), newNode);
        if (loadName.empty())
            newNode >> value;
        else
            newNode[loadName] >> value;
    }
    else
        node >> value;
}


class YamlDumpable
{
public:
    inline virtual ~YamlDumpable() { }
    virtual void dump(YAML::Emitter &out) const = 0;
};

template<typename T>
class YamlDumpableT
{
public:
    friend YAML::Emitter &operator<<(YAML::Emitter &out, const T &obj)
    {
        obj.dump(out);
        return out;
    }
};

class YamlLoadable
{
public:
    inline virtual ~YamlLoadable() { }
    virtual void load(const YAML::Node &in) = 0;
};

// Just do pure templates?
template<typename T>
class YamlLoadableT : public YamlLoadable
{
public:
    friend void operator>>(const YAML::Node &in, T &obj)
    {
        obj.load(in);
    }
};

} // namespace yaml
} // namespace addons
} // namespace controlit


namespace YAML {

Emitter &operator<<(Emitter &out, const boost::filesystem::path &path);

void operator>>(const Node &in, boost::filesystem::path &path);
Emitter &operator<<(Emitter &out, const boost::filesystem::path &path);

Emitter &operator<<(Emitter &out, const controlit::addons::yaml::YamlDumpable &obj);
void operator>>(const Node &in, controlit::addons::yaml::YamlLoadable &obj);

} // namespace YAML

std::ostream &operator<<(std::ostream &os, const YAML::Emitter &out);
std::ostream &operator<<(std::ostream &os, const controlit::addons::yaml::YamlDumpable &obj);

#endif // __CONTROLIT_DEPENDENCY_ADDONS_UTILITIES__
