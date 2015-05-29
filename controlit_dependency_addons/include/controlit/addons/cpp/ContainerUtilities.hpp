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

#ifndef __CONTROLIT_DEPENDENCY_ADDONS_CPP_CONTAINER_UTILITIES_HPP_
#define __CONTROLIT_DEPENDENCY_ADDONS_CPP_CONTAINER_UTILITIES_HPP_

#include <algorithm>
#include <vector>

namespace std {

// Qt-style vector stuff
// Generalize for other push_back'able containers?
template<typename T, typename Item>
inline vector<T>& operator<<(vector<T> &v, const Item &item)
{
    v.push_back(item);
    return v;
}

template<typename T, typename Item>
inline vector<T>& operator<<(vector<T> &v, const vector<Item> &other)
{
    std::copy(other.begin(), other.end(), back_inserter(v));
    return v;
}

} // namespace std


namespace controlit {
namespace addons {
namespace cpp {

template<typename T>
void vector_difference(const std::vector<T> &a, const std::vector<T> &b, std::vector<T> &out)
{
    using namespace std;
    // Carlos found out that these should be sorted
    vector<T> aCopy(a), bCopy(b);
    sort(aCopy.begin(), aCopy.end());
    sort(bCopy.begin(), bCopy.end());
    set_difference(aCopy.begin(), aCopy.end(), bCopy.begin(), bCopy.end(), back_inserter(out));
}

/*!
 * Computes the average of a vector of numbers
 *
 * \param[in] data The vector containing the double values.
 * \return The average
 */
template<typename T>
T average(const std::vector<T>& data)
{
    T sum = std::accumulate(data.begin(), data.end(), 0.0);
    return sum / data.size();
}

/*!
 * Computes the standard deviation of a vector of numbers
 *
 * \param[in] data The vector containing the double values.
 * \return The standard deviation
 */
template<typename T>
T stdev(const std::vector<T>& data)
{
    T avg = average(data);

    T accum = 0.0;
    std::for_each (std::begin(data), std::end(data), [&](const T d) {
        accum += (d - avg) * (d - avg);
    });

    return sqrt(accum / (data.size()-1));
}

} // namespace cpp
} // namespace addons
} // namespace controlit

#endif // __CONTROLIT_DEPENDENCY_ADDONS_CPP_CONTAINER_UTILITIES_HPP_
