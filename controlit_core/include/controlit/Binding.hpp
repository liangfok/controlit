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

#ifndef __CONTROLIT_CORE_BINDING_HPP__
#define __CONTROLIT_CORE_BINDING_HPP__

#include <controlit/Parameter.hpp>
#include <controlit/BindingConfig.hpp>

namespace controlit {

/*!
 * This is the parent class of all bindings.  Child classes implement transport-specific bindings.
 */
class Binding
{
public:
    /*!
     * The constructor.
     */
    Binding(Parameter * param, const BindingConfig & config);

    /*!
     * The destructor.
     */
    virtual ~Binding();

    /*!
     * Determines whether this binding is for a particular parameter.
     * This is determined by pointer comparisions.
     *
     * \return true if this binding is for the specified parameter.
     */
    bool isForParameter(const Parameter * param) const;

    /*!
     * \return A string representation of this class.
     */
    std::string toString() const;

protected:
    /*!
     * The parameter to be bound.
     */
    Parameter * param;

    /*!
     * The binding configuration.
     */
    BindingConfig config;
};

inline std::ostream &operator<<(std::ostream &out, const Binding & binding)
{
    out << "Parameter name: " << binding.toString();
    return out;
}

} // namespace controlit

#endif // __CONTROLIT_CORE_BINDING_HPP__
