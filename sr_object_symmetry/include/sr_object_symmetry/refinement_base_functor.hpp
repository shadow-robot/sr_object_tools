/**
* Copyright 2019 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation version 2 of the License.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program. If not, see <http://www.gnu.org/licenses/>.
*/



#ifndef SR_OBJECT_SYMMETRY_REFINEMENT_BASE_FUNCTOR_H
#define SR_OBJECT_SYMMETRY_REFINEMENT_BASE_FUNCTOR_H

// Eigen
#include "unsupported/Eigen/NonLinearOptimization"

namespace sym
{
/** \brief Base functor for non-linear optimization with Eigen. All the models
   * that need non linear optimization must define their own one and implement
   * either of:
   *   operator() (const Eigen::VectorXd& x, Eigen::VectorXd& fvec)
   *   operator() (const Eigen::VectorXf& x, Eigen::VectorXf& fvec)
   * dependening on the choosen _Scalar
   */
template <typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct BaseFunctor
{
  typedef _Scalar Scalar;

  enum
  {
    InputsAtCompileTime = NX,
    ValuesAtCompileTime = NY
  };

  typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
  typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
  typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

  int m_inputs, m_values;

  BaseFunctor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime)
  {
  }
  BaseFunctor(int inputs, int values) : m_inputs(inputs), m_values(values)
  {
  }

  int inputs() const
  {
    return m_inputs;
  }
  int values() const
  {
    return m_values;
  }
};
}  // namespace sym
#endif  // SR_OBJECT_SYMMETRY_REFINEMENT_BASE_FUNCTOR_H
