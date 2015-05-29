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

#ifndef __CONTROLIT_CORE_UNIQUE_OBJECT_HPP__
#define __CONTROLIT_CORE_UNIQUE_OBJECT_HPP__

#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators

namespace controlit {

class UniqueObject
{
public:
  UniqueObject() : _id(boost::uuids::random_generator()()) {};
  inline boost::uuids::uuid id() const {return _id;}

private:
  boost::uuids::uuid _id;
};

} // namespace controlit

#endif // __CONTROLIT_CORE_UNIQUE_OBJECT_HPP__
