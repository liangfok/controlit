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

#include <gtest/gtest.h>

#include <controlit/logging/logging.hpp>

using namespace std;

TEST(controlit_logging, create_logger_test)
{
    controlit::logging::LogStream<>* log = new controlit::logging::LogStream<>(drc::logging::Priority::Warning);
    ASSERT_NO_THROW(delete log);
}

TEST(controlit_logging, operator_test)
{
    string out(((std::stringbuf*)(DRC_WARN << 5 << ' ' << "aoeu " << string("xyz")).rdbuf())->str());
    ASSERT_EQ(out, "5 aoeu xyz");
}

TEST(controlit_logging, field_test)
{
    controlit::logging::LogStream<> log(drc::logging::Priority::Warning);
    log("package", ROS_PACKAGE_NAME);
    log << "foo";
    CONTROLIT_LOG(drc::logging::Priority::Warning);
    CONTROLIT_LOG(drc::logging::Priority::Warning) << "stuff";
    CONTROLIT_DEBUG << "This is a debug statement.";
}

