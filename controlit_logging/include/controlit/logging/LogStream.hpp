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

#ifndef __CONTROLIT_LOGGING_LOG_STREAM_HPP__
#define __CONTROLIT_LOGGING_LOG_STREAM_HPP__

#include <chrono>
#include <sstream>
#include <iostream>
#include <ctime>
#include <ostream>
#include <mutex>
#include <memory>
#include <unistd.h> // for getpid()
#include <vector>
#include <algorithm>

#include <ros/ros.h>  // for ROS logging macros

namespace controlit {
namespace logging {

enum class Priority : unsigned int
{
    // Information that you never need to see if the system is working properly.
    Debug = 0,
    // Small amounts of information that may be useful to a user.
    Info = 1,
    // Information that the user may find alarming, and may affect the output of the
    // application, but is part of the expected working of the system.
    Warning = 2,
    // Something serious (but recoverable) has gone wrong.
    Error = 3,
    // Something unrecoverable has happened. Initiate a safe-shutdown
    Fatal = 4
};

// declare externally-defined variables
extern __thread bool _init_called;
extern Priority _log_level;
extern std::vector<std::string> _fields_to_print;

void _init(std::string const& srcdir, std::string const& pkgname);

template <
    class charT = char,
    class traits = std::char_traits<charT>,
    class Alloc = std::allocator<charT>
>
class LogStream : public std::basic_ostream<charT, traits>
{
    class LogBuf : public std::basic_stringbuf<charT, traits, Alloc>
    {
        // Json::FastWriter json;
        Priority priority;
        bool sendOnDestroy;

        unsigned int priorityToUnsignedInt(const Priority priority)
        {
            switch(priority)
            {
                case(Priority::Debug):   return 0;
                case(Priority::Info):    return 1;
                case(Priority::Warning): return 2;
                case(Priority::Error):   return 3;
                case(Priority::Fatal):   return 4;
            }
            return std::numeric_limits<unsigned int>::max();
        }

        std::string priorityToString(Priority priority)
        {
            switch (priority)
            {
                case Priority::Debug:   return "DEBUG";
                case Priority::Info:    return "INFO";
                case Priority::Warning: return "WARN";
                case Priority::Error:   return "ERROR";
                case Priority::Fatal:   return "FATAL";
            }
            return "UNKNOWN"; // should never get here
        }

        void printFields(std::stringstream & sstr)
        {
            bool firstProperty = true; // whether to prefix a property with a comma

            for (size_t n = 0; n < fields.size(); n++)
            {
                // Only print the fields that are in the _fields_to_print vector
                if (std::find(_fields_to_print.begin(), _fields_to_print.end(), fields[n].first) != _fields_to_print.end())
                {
                    if (fields[n].first == "file")
                    {
                        if (!firstProperty) sstr << " "; // add a space if property is not the first in the list
                        firstProperty = false;

                        // Get the position of the last forward slash in the file name
                        size_t posOfSlash = fields[n].second.find_last_of("/");

                        sstr << "[";

                        // If the file name includes the entire path to it, omit the path.
                        if (posOfSlash != fields[n].second.npos && posOfSlash != fields[n].second.size())
                            sstr << fields[n].first << ": " << fields[n].second.substr(posOfSlash + 1);
                        else
                            sstr << fields[n].first << ": " << fields[n].second;

                        sstr << "]";
                    }
                    else if (fields[n].first == "function")
                    {
                        if (!firstProperty) sstr << " "; // add a space if property is not the first in the list
                        firstProperty = false;

                        // Get the position of the last double colon in the function name
                        size_t posOfDoubleColon = fields[n].second.find_last_of("::",
                            fields[n].second.find_last_of("("));

                        sstr << "[";

                        // If the function name includes the entire namespace, omit the name space.
                        if (posOfDoubleColon != fields[n].second.npos && posOfDoubleColon != fields[n].second.size())
                            sstr << fields[n].first << ": " << fields[n].second.substr(posOfDoubleColon + 1);
                        else
                            sstr << fields[n].first << ": " << fields[n].second;

                        sstr << "]";
                    }
                    else if (fields[n].first == "pid")
                    {
                        if (!firstProperty) sstr << " "; // add a space if property is not the first in the list
                        firstProperty = false;


                        sstr << "[pid: " << getpid() << "]";
                    }
                    else
                    {
                        if (!firstProperty) sstr << " "; // add a space if property is not the first in the list
                        firstProperty = false;

                        sstr << "[" << fields[n].first << ": " << fields[n].second << "]";
                    }
                }
            }
        }

    public:
        std::vector<std::pair<std::string, std::string>, Alloc> fields;

        /*!
         * The constructor.
         *
         * \param p The priority level of the message.
         * \param sendOnDestroy Whether to print the message upon destruction of this method.
         */
        LogBuf(Priority p, bool sendOnDestroy) : priority(p), sendOnDestroy(sendOnDestroy) {}

        /*!
         * The destructor.
         */
        ~LogBuf()
        {
            // Abort if sendOnDestroy is false
            if(!sendOnDestroy)
                return;

            // Calls the sync method, see: http://en.cppreference.com/w/cpp/io/basic_streambuf/pubsync
            std::basic_stringbuf<charT, traits, Alloc>::pubsync();
        }

        int sync()
        {
            // static const char TEXT_COLOR_NORMAL[] = "\033[0m";
            // static const char TEXT_COLOR_WARN[] = "\033[33m";
            // static const char TEXT_COLOR_ERROR[] = "\033[31m";
            // static const char TEXT_COLOR_FATAL[] = "\033[31;1m";

            // std::cout << "Checking priority level!\n"
            //   << "  - priority: " << priorityToString(priority) << "\n"
            //   << "  - log level: " << priorityToString(_log_level) << std::endl;

            if(priority >= _log_level)
            {
                // build message
                // Json::Value root;

                // get the current time in floating point seconds
                // auto timestamp = std::chrono::system_clock::now();
                // auto duration = timestamp.time_since_epoch();
                // auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
                // root["timestamp"] = millis * 0.001f;
                // root["message"] = std::basic_stringbuf<charT, traits, Alloc>::str();

                // for(auto& f : fields)
                //     root[f.first] = f.second;

                // send message
                // std::string raw = json.write(root);
                // zmq::message_t msg(raw.size());
                // memcpy(msg.data(), raw.c_str(), raw.size());
                // _socket->send(msg, ZMQ_NOBLOCK);  // don't block if there is no receiver

                // Construct pretty console output
                // TODO: Move this console printing out to a specialized buffer? Shouldn't
                // be in-line with zmq stuff and certainly should be an option in the configuration.
                // TODO: Absolute IDEAL way to do this would be through custom formatter
                // class. Take a look at how Eigen implements its custom formaters.
                {
                    std::stringstream sstr;

                    // switch (priority)
                    // {
                    //     case Priority::Debug:
                    //         sstr << TEXT_COLOR_NORMAL << "[DEBUG]"; break;
                    //     case Priority::Info:
                    //         sstr << TEXT_COLOR_NORMAL << "[INFO]"; break;
                    //     case Priority::Warning:
                    //         sstr << TEXT_COLOR_WARN << "[WARN]"; break;
                    //     case Priority::Error:
                    //         sstr << TEXT_COLOR_ERROR << "[ERROR]"; break;
                    //     case Priority::Fatal:
                    //         sstr << TEXT_COLOR_FATAL << "[FATAL]"; break;
                    // }

                    // Get time stamp
                    auto timestamp = std::chrono::system_clock::now();
                    std::time_t tt = std::chrono::system_clock::to_time_t(timestamp);
                    //std::tm tm = *std::localtime(&time_stamp);
                    char time_stamp_str[255];
                    std::strftime(time_stamp_str, 255, "%H:%M:%S", std::localtime(&tt));
                    sstr << " [" << time_stamp_str << "] ";

                    // Add the fields
                    printFields(sstr);

                    // Add the message
                    sstr << ": " << std::basic_stringbuf<charT, traits, Alloc>::str() << std::endl;

                    // debug print to cout.. and reset the color in case some is using cout
                    // std::cout << sstr.str() << TEXT_COLOR_NORMAL;

                    switch (priority)
                    {
                        case Priority::Debug:
                            ROS_DEBUG_STREAM(sstr.str()); break;
                        case Priority::Info:
                            ROS_INFO_STREAM(sstr.str()); break;
                        case Priority::Warning:
                            ROS_WARN_STREAM(sstr.str()); break;
                        case Priority::Error:
                            ROS_ERROR_STREAM(sstr.str()); break;
                        case Priority::Fatal:
                            ROS_FATAL_STREAM(sstr.str()); break;
                    }

                }

                // clear the stream
                std::basic_stringbuf<charT, traits, Alloc>::str("");
            }
            else
            {
                std::cout << "priority is less than _log_level" << std::endl;
            }
            // else
            // {
            //     if (_print_suppressed_msgs)
            //     {
            //         std::stringstream sstr;
            //         sstr << "[SUPPRESSED] ";

            //         auto timestamp = std::chrono::system_clock::now();
            //         std::time_t tt = std::chrono::system_clock::to_time_t(timestamp);
            //         //std::tm tm = *std::localtime(&time_stamp);
            //         char time_stamp_str[255];
            //         std::strftime(time_stamp_str, 255, "%H:%M:%S", std::localtime(&tt));
            //         sstr << "[" << time_stamp_str << "] ";

            //         sstr << "[priority: " << priorityToString(priority)
            //              << "] [log level: " << priorityToString(_log_level) << "] ";
            //         printFields(sstr);
            //         sstr << ": " << std::basic_stringbuf<charT, traits, Alloc>::str()
            //              << std::endl;
            //         std::cout  << TEXT_COLOR_WARN << sstr.str() << TEXT_COLOR_NORMAL;
            //     }
            // }
            return 0;
        }
    };
public:

    /*!
     * The constructor.
     *
     * \param[in] priority The priority level of the log message.
     * \param[in] sendOnDestroy Whether to send the log message when this object is destroyed.
     */
    LogStream(Priority priority = Priority::Info, bool sendOnDestroy = true)
        : std::ostream(new LogBuf(priority, sendOnDestroy)) { }

    /*!
     * The destructor.  It send the log when this log stream leaves scope.
     */
    ~LogStream()
    {
        if(not _init_called)
            _init(std::string(PROJECT_SOURCE_DIR), std::string(PACKAGE_NAME));
        //controlit_assert_msg(_init_called, "Log sent before calling init!");

        delete std::basic_ostream<charT, traits>::rdbuf();
    }

    template <class T, class U>
    LogStream& operator()(const T& name, const U& value)
    {
        auto a = (LogBuf*)std::basic_ostream<charT, traits>::rdbuf();
        a->fields.emplace_back(name, value);
        return *this;
    }
};

}; // namespace logging
}; // namespace controlit

#endif // __CONTROLIT_LOGGING_LOG_STREAM_HPP__
