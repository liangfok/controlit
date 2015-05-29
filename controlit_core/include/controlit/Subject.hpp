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

#ifndef BOOST_SIGNALS_NO_DEPRECATION_WARNING
#define BOOST_SIGNALS_NO_DEPRECATION_WARNING
#endif

#ifndef __CONTROLIT_CORE_SUBJECT_HPP__
#define __CONTROLIT_CORE_SUBJECT_HPP__

#include <map>
#include <boost/uuid/uuid.hpp>
#include <boost/signal.hpp>
#include <boost/bind.hpp>

#include <controlit/UniqueObject.hpp>

namespace controlit {

template<class Listener>
class TypedSubject
{
    typedef typename Listener::SignalArgument_t SignalArgument_t;
    typedef typename Listener::SignalPrototype_t SignalPrototype_t;
    typedef boost::signal<SignalPrototype_t>  Signal_t;
    Signal_t signal;

public:
    typedef boost::signals::connection  Connection_t;
    typedef boost::function<SignalPrototype_t> Functor_t;

    Connection_t addListener(Functor_t functor)
    {
        Connection_t connection = signal.connect(functor);
        return connection;
    }

protected:
    void notifyListeners(SignalArgument_t const& arg)
    {
        signal(arg);
    }
};

} // namespace controlit

#endif //__CONTROLIT_CORE_SUBJECT_HPP__
