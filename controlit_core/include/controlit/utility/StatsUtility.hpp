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

#ifndef __CONTROLIT_STATS_UTILITY_UTILITY_H__
#define __CONTROLIT_STATS_UTILITY_UTILITY_H__

#include <vector> // for std::vector
#include <numeric> // for std::accumulate
#include <algorithm> // for std::transform
#include <functional> // for bind2nd
#include <cmath> // for std::sqrt

namespace controlit {
namespace utility {

void computeAvgAndStdDev(const std::vector<double> & data, double &avg, double &std)
{
    double sum;
    computeAvgAndStdDev(data, sum, avg, std);
}

void computeAvgAndStdDev(const std::vector<double> & data, double &sum, double &avg, double &std)
{ 
    sum = std::accumulate(data.begin(), data.end(), 0.0);
    avg = sum / data.size();
  
    std::vector<double> diff(data.size());
    std::transform(data.begin(), data.end(), diff.begin(), std::bind2nd(std::minus<double>(), avg));
    double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    std = std::sqrt(sq_sum / data.size());
}

} // namespace utility
} // namespace controlit

#endif
