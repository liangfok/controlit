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

#ifndef __CONTROLIT_CORE_SINGLE_THREADED_CONTROL_MODEL_HPP__
#define __CONTROLIT_CORE_SINGLE_THREADED_CONTROL_MODEL_HPP__

#include <controlit/RTControlModel.hpp>

namespace controlit {

/*!
 * Extends the RTControlModel but does not implement a child thread
 * to update the control model.  Instead, the control model is updated
 * with each call to get().  This is intended to be used for
 * benchmarking purposes when comparing multi-threaded vs. single-threaded
 * implementations of ControlModel.
 */
class SingleThreadedControlModel : public RTControlModel
{
public:
    /*!
     * The constructor.
     */
    SingleThreadedControlModel();

    /*!
     * The destructor.
     */
    virtual ~SingleThreadedControlModel();

    /*!
     * Sets the A matrix mask of all control models used internally within this class.
     *
     * \param mask The A matrix mask.
     */
    virtual void setAMask(std::vector<std::vector<std::string>> mask);

    /*!
     * Sets the gravity mask of all control models used internally within this class.
     *
     * \param mask The gravity vector mask.
     */
    virtual void setGravMask(std::vector<std::string> mask);

    /*!
     * Overrides the parent class' method.  Doesn't do anything
     * since there's only one thread in this implementation.
     */
    virtual void startThread();

    /*!
     * Overrides the parent class' method.  Doesn't do anything
     * since there's only one thread in this implementation.
     */
    virtual void stopThread();

    /*!
     * Overrides the parent class' method.  Always returns a pointer
     * to the inactive control model since there's no lock to obtain.
     */
    virtual ControlModel * trylock();

    /*!
     * Overrides the parent class' method.  Always returns true to
     * pretend as if a new model was available everytime this method
     * is called.
     */
    virtual bool checkUpdate();

    /*!
     * Overrides the parent class' method.  It always updates
     * the control model.
     */
    virtual void unlockAndUpdate();
};

} // namespace controlit

#endif
