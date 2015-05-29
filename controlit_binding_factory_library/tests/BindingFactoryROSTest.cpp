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

#include <ros/ros.h>
#include <gtest/gtest.h>

#include <controlit/binding_factory_library/BindingFactoryROS.hpp>
#include <controlit/BindingFactory.hpp>
#include <controlit/Binding.hpp>
#include <controlit/BindingConfig.hpp>
#include <controlit/Parameter.hpp>

namespace controlit {
namespace binding_factory_library {

class BindingFactoryROSTest : public ::testing::Test
{
protected:

    virtual void SetUp()
    {
        bindingFactory.reset(new BindingFactoryROS());
    }

    virtual void TearDown()
    {
        bindingFactory.reset();
    }

    std::unique_ptr<controlit::BindingFactory> bindingFactory;
};

// Test to verify a newly created Binding can be deleted using a pointer to the parent class.
TEST_F(BindingFactoryROSTest, BasicTest)
{
    std::string transportType = "ROSTopic";
    std::string transportDataType = "std_msgs/Float64MultiArray";
    std::string topic = "JPosTask/error";
    controlit::BindingConfig::Direction direction = controlit::BindingConfig::Input;

    BindingConfig bindingConfig(transportType, transportDataType, topic, direction);

    int argc = 0;
    ros::init(argc, NULL, "BindingFactoryROSTest");

    ros::NodeHandle nh;

    std::string parameterName = "MyParameter";
    ParameterType parameterType = controlit::PARAMETER_TYPE_VECTOR;
    unsigned int flags = 0;

    Parameter param(parameterName, parameterType, flags);

    Binding * binding = bindingFactory->createBinding(nh, bindingConfig, &param);
    std::cout << "Created binding." << std::endl;

    EXPECT_TRUE(binding);

    std::cout << "Deleting newly created binding..." << std::endl;
    delete binding;

    std::cout << "Done deleting binding." << std::endl;
}

} // namespace binding_factory_library
} // namespace controlit
