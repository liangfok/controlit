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

#include <fstream>
#include <sstream>
#include <stdexcept>

#include <controlit/addons/yaml/yaml_utilities.hpp>

using namespace YAML;
using namespace std;

namespace controlit {
namespace addons {
namespace yaml {

void yaml_read_stream(std::istream &stream, YAML::Node &doc)
{
    YAML::Parser parser(stream);
    parser.GetNextDocument(doc);
}

void yaml_read_file(boost::filesystem::path const &filePath, YAML::Node &doc)
{
    if (!boost::filesystem::exists(filePath))
    {
        ostringstream os;
        os << "yaml_read_file: file '" << filePath << "' does not exist";
        throw std::runtime_error(os.str());
    }

    std::ifstream fin(filePath.string().c_str());
    yaml_read_stream(fin, doc);
    fin.close();
}

void yaml_read_string(std::string const &str, YAML::Node &doc)
{
    std::istringstream sstr(str);
    yaml_read_stream(sstr, doc);
}

void yaml_write_stream(std::ostream &stream, YAML::Emitter const &out)
{
    stream << out.c_str();
}

void yaml_write_file(boost::filesystem::path const &filePath, YAML::Emitter const &out)
{
    std::ofstream fout(filePath.string().c_str());
    yaml_write_stream(fout, out);
    fout.close();
}

void yaml_write_string(std::string &str, YAML::Emitter const &out)
{
    std::ostringstream ss;
    yaml_write_stream(ss, out);
    str = ss.str();
}

void yaml_read_stdin(Node &doc, bool ignoreFirst)
{
    if (ignoreFirst)
        cin.ignore();
    stringbuf buffer;
    cin.get(buffer);

    istream line(&buffer);
    Parser parser(line);
    parser.GetNextDocument(doc);
}

} // namespace yaml
} // namespace addons
} // namespace controlit

// using namespace controlit::addons::yaml;

namespace YAML {

void operator>>(const Node &in, boost::filesystem::path &path)
{
    string str;
    in >> str;
    path = str;
}

Emitter &operator<<(Emitter &out, const boost::filesystem::path &path)
{
    return out << path.string();
}

void operator>>(const Node &in, controlit::addons::yaml::YamlLoadable &obj)
{
    obj.load(in);
}

Emitter &operator<<(Emitter &out, const controlit::addons::yaml::YamlDumpable &obj)
{
    obj.dump(out);
    return out;
}

} // namespace YAML


std::ostream &operator<<(std::ostream &os, const YAML::Emitter &out)
{
    return os << out.c_str();
}

std::ostream &operator<<(std::ostream &os, const controlit::addons::yaml::YamlDumpable &obj)
{
    Emitter out;
    out << obj;
    return os << out;
}
