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

#ifndef __CONTROLIT_CONSTRAINT_LIBRARY_FLAT_CONTACT_CONSTRAINT_HPP__
#define __CONTROLIT_CONSTRAINT_LIBRARY_FLAT_CONTACT_CONSTRAINT_HPP__

#include <controlit/Constraint.hpp>

namespace controlit {
namespace constraint_library {

class FlatContactConstraint : public Constraint
{
public:
    /*!
     * The constructor.
     */
    FlatContactConstraint();
  
    /*!
     * The destructur.
     */
    virtual ~FlatContactConstraint() {}
  
    /*!
     * Initializes this constraint.
     *
     * \param[in] robot The robot model.
     */
    virtual void init(RigidBodyDynamics::Model& robot);
  
    /*!
     * Obtains this constraint's jacobian matrix.
     *
     * \param[in] robot The robot model.
     * \param[in] Q The current joint state.
     * \param[out] Jc The constraint jacobian.
     */
    virtual void getJacobian(RigidBodyDynamics::Model& robot, const Vector& Q, Matrix& Jc);

protected:
    /*!
     * Sets up the parameters.
     */
    virtual void setupParameters();
  
    //! Contact point in body frame
    Vector cp1_;
  
    //! Storage for temporary calculations
    Matrix Jpv, Jpw;
};

} // namespace constraint_library
} // namespace controlit

#endif
