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

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/positional_options.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/parsers.hpp>

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <controlit_robot_models/rbdl_robot_urdfreader.hpp>

#include <iostream>
#include <map>

namespace po = boost::program_options;

void usage()
{
    std::cout << "Usage: check_model [options] filename.urdf" << std::endl;
}

int main (int argc, char *argv[])
{
    // Input file to parse
    std::string filename;

    // Generic options
    po::options_description generic("Generic options");
    generic.add_options()
        ("help",                "produce help message")
        ("verbose,v",           "enable additional output")
    ;

    // Configuration options
    po::options_description config("Configuration");
    config.add_options()
        ("dof-overview,d",      "print an overview of the degress of freedom")
        ("model-hierarchy,m",   "print the hierarchy of the model")
    ;

    // Hidden options
    po::options_description hidden("Hidden options");
    hidden.add_options()
        ("input-file",          po::value<std::string>(&filename),  "input file")
    ;

    // Declare the position options
    po::positional_options_description p;
    p.add("input-file", -1);

    // Concatenate options together
    po::options_description cmdline_options;
    cmdline_options.add(generic).add(config).add(hidden);

    po::options_description visible("Allowed options");
    visible.add(generic).add(config);

    // Parse command line arguments
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).
        options(cmdline_options).positional(p).run(), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        usage();
        std::cout << visible << std::endl;
        return 1;
    }

    if (!vm.count("input-file"))
    {
        std::cerr << "Please specify and input file." << std::endl;
        std::cout << std::endl;
        usage();
        return -1;
    }

    // Get verbosity
    bool verbose = false;
    if (vm.count("verbose")) verbose = true;


    RigidBodyDynamics::Model robot;
    // robot.Init();

    std::map<std::string, std::string> l2jmap;

    if (!controlit::rbdl_robot_urdfreader::read_urdf_model_from_file(filename, &robot, &l2jmap, verbose))
    {
        std::cerr << "Loading of urdf robot failed!" << std::endl;
        return -1;
    }

    std::cout << "Model successfully loaded from: " << filename << std::endl;

    if (vm.count("dof-overview"))
    {
        std::cout << std::endl << "Degree of freedom overview:" << std::endl;
        std::cout << RigidBodyDynamics::Utils::GetModelDOFOverview(robot);
    }

    if (vm.count("model-hierarchy"))
    {
        std::cout << std::endl << "Model Hierarchy:" << std::endl;
        std::cout << RigidBodyDynamics::Utils::GetModelHierarchy (robot);
    }

    return 1;
}