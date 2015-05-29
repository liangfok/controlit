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

#ifndef __CONTROLIT_ADDONS_CPP_STRING_UTILITIES_HPP__
#define __CONTROLIT_ADDONS_CPP_STRING_UTILITIES_HPP__

#include <boost/algorithm/string.hpp>

namespace controlit {
namespace addons {
namespace cpp {

struct string_splitter
{
    typedef std::vector<std::string> Elements;

    // Splits a string, s, into its components based on a delimiter, delim
    static void split(const std::string &s, char delim, Elements &elems)
    {
        std::stringstream ss(s);
        std::string item;
        while (std::getline(ss, item, delim))
        {
            elems.push_back(item);
        }
    }

    string_splitter(std::string const &str, char delim) : _str(str), _delim(delim) {};

    std::string next()
    {
        size_t d = _str.find(_delim,0);
        std::string item;
        //std::cout << "d: " << d << std::endl;
        if (d == std::string::npos)
        {
            item = _str;
            _str = "";
        }
        else
        {
            item = _str.substr(0,d);
            _str = _str.substr(d+1);
            //std::cout << "item: " << item << std::endl
            //		  << "_str: " << _str << std::endl;
        }
        return item;
    }

    std::string rest()
    {
        return _str;
    }

    std::string _str;
    char _delim;
};


// From: http://stackoverflow.com/a/5419348/170413
template<typename OutputIterator>
inline void split(const std::string &str, const std::string &delim, OutputIterator result)
{
    using namespace boost::algorithm;
    typedef split_iterator<std::string::const_iterator> Iterator;

    for(Iterator iter = make_split_iterator(str, first_finder(delim, is_equal())); iter != Iterator(); ++iter)
        *(result++) = boost::copy_range<std::string>(*iter);
}


inline void split(const std::string &str, const std::string &delim, std::vector<std::string> &result)
{
    split(str, delim, std::back_inserter(result));
}


inline std::vector<std::string> split(const std::string &str, const std::string &delim)
{
    std::vector<std::string> result;
    split(str, delim, std::back_inserter(result));
    return result;
}


class string_icomparator
{
    std::string value;
public:
    inline string_icomparator(const std::string &value)
        : value(value)
    { }
    inline bool operator()(const std::string &test)
    {
        return boost::algorithm::iequals(value, test);
    }
};

template<typename InputIt, typename T>
InputIt find_icase(InputIt first, InputIt last, const T &value)
{
    //How to use bind??? Does not work: boost::bind(boost::algorithm::iequals, _1, value));
    return std::find_if(first, last, string_icomparator(value));
}


} // namespace cpp
} // namepsace addons
} // namespace controlit

#endif // __CONTROLIT_ADDONS_CPP_STRING_UTILITIES_HPP__
