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

#ifndef __CONTROLIT_ADDONS_EIGEN_PSEUDO_INVERSE__
#define __CONTROLIT_ADDONS_EIGEN_PSEUDO_INVERSE__


#include <Eigen/Dense>
#include <controlit/addons/cpp/Assert.hpp>

namespace controlit {
namespace addons {
namespace eigen {

/**
 * Computes the pseudo inverse of a matrix, based on SVD
 * followed by thresholding on the singular values.
 *
 * \param[in] M The matrix to find the pseudo inverse of.
 * \param[out] Minv Where the results should be sored.
 * \param[in] epsilon Controls cutoff for small singular values
*/
template<typename DerivedA, typename OutputMatrixType>
// typename DerivedA::Scalar
void pseudo_inverse(const Eigen::MatrixBase<DerivedA>& M,
  Eigen::MatrixBase<OutputMatrixType>& Minv,
  typename DerivedA::Scalar epsilon = 1e-6)//std::numeric_limits<typename DerivedA::Scalar>::epsilon())
{
  // CONTROLIT_INFO << "Method called!\n  epsilon = " << epsilon << ", M = \n" << M;

  // Ensure matrix Minv has the correct size.  Its size should be equal to M.transpose().
  controlit_assert_msg(M.rows() == Minv.cols(), "Minv has invalid number of columns.  Expected " << M.rows() << " got " << Minv.cols());
  controlit_assert_msg(M.cols() == Minv.rows(), "Minv has invalid number of rows.  Expected " << M.cols() << " got " << Minv.rows());

  // According to Eigen documentation, "If the input matrix has inf or nan coefficients, the result of the
  // computation is undefined, but the computation is guaranteed to terminate in finite (and reasonable) time."
  Eigen::JacobiSVD<DerivedA> svd = M.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);

  // Get the max singular value
  typename DerivedA::Scalar maxSingularValue = svd.singularValues().array().abs().maxCoeff();

  // Use Minv to temporarily hold sigma
  Minv.setZero();

  typename DerivedA::Scalar tolerance = 0;

  // Only compute sigma if the max singular value is greater than zero.
  if (maxSingularValue > epsilon)
  {
    tolerance = epsilon * std::max(M.cols(), M.rows()) * maxSingularValue;

    // For each singular value of matrix M's SVD decomposition, check if it is greater than
    // the tolerance value.  If it is, save 1/(singular value) in the sigma vector.
    // Otherwise save zero in the sigma vector.
    DerivedA sigmaVector = DerivedA( (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0) );
    // DerivedA zeroSVs = DerivedA( (svd.singularValues().array().abs() <= tolerance).select(svd.singularValues().array().inverse(), 0) );

    // CONTROLIT_INFO << "epsilon: " << epsilon << ", std::max(M.cols(), M.rows()): " << std::max(M.cols(), M.rows()) << ", maxSingularValue: " << maxSingularValue << ", tolerance: " << tolerance;
    // CONTROLIT_INFO << "sigmaVector = " << sigmaVector.transpose();
    // CONTROLIT_INFO << "zeroSigmaVector : "<< zeroSVs.transpose();

    Minv.block(0, 0, sigmaVector.rows(), sigmaVector.rows()) = sigmaVector.asDiagonal();
  }

  // Double check to make sure the matrices have the correct dimensions
  controlit_assert_msg(svd.matrixV().cols() == Minv.rows(),
    "Matrix dimension mismatch, svd.matrixV().cols() = " << svd.matrixV().cols() << ", Minv.rows() = " << Minv.rows() << ".");
  controlit_assert_msg(Minv.cols() == svd.matrixU().adjoint().rows(),
    "Matrix dimension mismatch, Minv.cols() = " << Minv.cols() << ", svd.matrixU().adjoint().rows() = " << svd.matrixU().adjoint().rows() << ".");

  Minv = svd.matrixV() *
         Minv *
         svd.matrixU().adjoint(); // take the transpose of matrix U

  // CONTROLIT_INFO << "Done method call! Minv = " << Minv;

  // typename DerivedA::Scalar errorNorm = std::abs((M * Minv - DerivedA::Identity(M.rows(), Minv.cols())).norm());

  // if (tolerance != 0 && errorNorm > tolerance * 10)
  // {
  //   CONTROLIT_WARN << "Problems computing pseudoinverse.  Perhaps the tolerance is too high?\n"
  //     << "  - epsilon: " << epsilon << "\n"
  //     << "  - tolerance: " << tolerance << "\n"
  //     << "  - maxSingularValue: " << maxSingularValue << "\n"
  //     << "  - errorNorm: " << errorNorm << "\n"
  //     << "  - M:\n" << M << "\n"
  //     << "  - Minv:\n" << Minv;
  // }

  // return errorNorm;
}

} // namespace eigen
} // namespace addons
} // namespace controlit

#endif
