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

#ifndef __CONTROLIT_CORE_PARAMETER_LOG_HPP__
#define __CONTROLIT_CORE_PARAMETER_LOG_HPP__

#include <vector>

#include <controlit/Parameter.hpp>

namespace controlit {

class ParameterLog
{
public:
  template<typename parameter_t, typename storage_t>
  struct log_s
  {
    explicit log_s(parameter_t const* pp): parameter(pp) {}
    parameter_t const* parameter;
    std::vector<storage_t> log;
  };

  ParameterLog(std::string const& name, ParameterMap const& parameter_lookup);

  void update(long long timestamp);
  void writeFiles(std::string const& prefix, std::ostream* progress) const;

  std::string const name;
  std::vector<long long> timestamp;
  std::vector<log_s<IntegerParameter, int> > intlog;
  std::vector<log_s<StringParameter, std::string> > strlog;
  std::vector<log_s<RealParameter, double> > reallog;
  std::vector<log_s<VectorParameter, Vector> > veclog;
  std::vector<log_s<MatrixParameter, Matrix> > mxlog;
};

} // end namespace controlit

#endif // __CONTROLIT_CORE_PARAMETER_LOG_HPP__
