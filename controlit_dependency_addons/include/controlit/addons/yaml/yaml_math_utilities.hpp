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

#ifndef __CONTROLIT_DEPENDENCY_ADDONS_YAML_MATH_UTILITIES__
#define __CONTROLIT_DEPENDENCY_ADDONS_YAML_MATH_UTILITIES__

#include <sstream>

#include <Eigen/Core>

#include <controlit/addons/cpp/Assert.hpp>
#include <controlit/addons/yaml/yaml_utilities.hpp>

// Have to put these here to overload yaml-cpp's implementation for vector in stlnode
namespace YAML {

typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

// Reading

/*!
 * Resize an Eigen vector to match the number of elements contained within a YAML node.
 *
 * \param[in] node The YAML node.
 * \param[dest] dest The Eigen matrix to resize.
 */
template<typename Derived>
void eigen_resize_vector(const Node & node, Eigen::MatrixBase<Derived> & dest)
{
    // http://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html
    dest.derived().resize(node.size());
}

template<typename Derived>
void eigen_read_vector(const Node &in, Eigen::MatrixBase<Derived> &dest)
{
    controlit_assert_msg(dest.size() == (long)in.size(), "Size not equal. Eigen = " << dest.size() << ", YAML = " << in.size());

    for (int ii = 0; ii < dest.size(); ++ii)
        in[ii] >> dest(ii);
}

template<typename Derived>
void eigen_resize_matrix(const Node &in, Eigen::MatrixBase<Derived> &X)
{
    uint rows = in.size();
    if (rows > 0)
    {
        // If just one row, then resize to column vector
        uint cols = in[0].size();
        if (cols == 0)
            X.derived().resize(rows, 1);
        else
            X.derived().resize(rows, in[0].size());
    }
    else
        X.derived().resize(0, 0);
}

template<typename Derived>
void eigen_read_matrix(const Node &in, Eigen::MatrixBase<Derived> &X)
{
    typedef Eigen::MatrixBase<Derived> Matrix;
    int rows = (int)in.size();
    if (rows > 0)
    {
        int cols = (int)in[0].size();
        // Load as a vector if it's a single list
        if (cols == 0)
            eigen_read_vector(in, X);
        else
        {
            controlit_assert_msg(X.rows() == rows, "Rows not equal. Eigen = " << X.rows() << ", YAML = " << rows);
            controlit_assert_msg(X.cols() == cols, "Cols not equal. Eigen = " << X.cols() << ", YAML = " << cols);
            for (int ii = 0; ii < rows; ++ii)
            {
                typename Matrix::RowXpr row = X.row(ii);
                eigen_read_vector(in[ii], row);
            }
        }
    }
}


// Writing

template<typename Derived>
void eigen_write_vector(Emitter &out, const Eigen::MatrixBase<Derived> &X)
{
    out << Flow << BeginSeq;
    for (uint ii = 0; ii < (uint)X.size(); ++ii)
        out << X.coeff(ii);
    out << EndSeq;
}

template<typename Derived>
void eigen_write_matrix(Emitter &out, const Eigen::MatrixBase<Derived> &X)
{
    out << BeginSeq;
    for (uint ii = 0; ii < (uint)X.rows(); ++ii)
        eigen_write_vector(out, X.row(ii));
    out << EndSeq;
}

// Piecing it together

template<typename Derived, int IsVector>
struct eigen_handler_impl
{ };

// Vector specialization
template<typename Derived>
struct eigen_handler_impl<Derived, 1>
{
    typedef Eigen::MatrixBase<Derived> Matrix;
    static void resize(const Node &in, Matrix &X)
    {
        eigen_resize_vector(in, X);
    }

    static void read(const Node &in, Matrix &X)
    {
        eigen_read_vector(in, X);
    }

    static void write(Emitter &out, const Matrix &X)
    {
        eigen_write_vector(out, X);
    }
};

// Matrix specialization
template<typename Derived>
struct eigen_handler_impl<Derived, 0>
{
    typedef Eigen::MatrixBase<Derived> Matrix;
    static void resize(const Node &in, Matrix &X)
    {
        eigen_resize_matrix(in, X);
    }

    static void read(const Node &in, Matrix &X)
    {
        eigen_read_matrix(in, X);
    }

    static void write(Emitter &out, const Matrix &X)
    {
        eigen_write_matrix(out, X);
    }
};

// Make it easy to select
template<typename Derived>
struct eigen_handler : public eigen_handler_impl<Derived, Eigen::MatrixBase<Derived>::IsVectorAtCompileTime>
{ };

template<typename Derived>
void eigen_read_and_resize(const Node &in, Eigen::MatrixBase<Derived> &X)
{
    typedef eigen_handler<Derived> handler;
    if (handler::Matrix::SizeAtCompileTime == Eigen::Dynamic)
        handler::resize(in, X);
    handler::read(in, X);
}


// YAML has trouble with the templating stuff, just going to overload some specific matrix here
// @todo Look into this: http://code.google.com/p/yaml-cpp/issues/detail?id=150#c13
template<typename Scalar, int Cols, int Rows>
void operator>>(const Node &in, Eigen::Matrix<Scalar, Cols, Rows> &X)
{
    eigen_read_and_resize(in, X);
}

template<typename Derived>
Emitter &operator<<(Emitter &out, const Eigen::MatrixBase<Derived> &X)
{
    typedef eigen_handler<Derived> handler;
    handler::write(out, X);
    return out;
}

//template <typename Derived>
//inline bool Node::Read(Eigen::MatrixBase<Derived>& value) const
//{
//    eigen_read_and_resize(*this, value);
//}

//template<typename Derived>
//struct convert<Eigen::MatrixBase<Derived> >
//{
//    typedef Eigen::MatrixBase<Derived> Matrix;
//    static Node encode(const Matrix& rhs) {
//        Node node;
//        eigen_handler<Derived>::write(node);
//        return node;
//    }

//    static bool decode(const Node& node, const Eigen::MatrixBase<Derived>& rhs) {
//        eigen_read_and_resize(node, rhs);
//        return true;
//    }
//};

} // End namespace YAML

#endif  // __CONTROLIT_DEPENDENCY_ADDONS_YAML_MATH_UTILITIES__
