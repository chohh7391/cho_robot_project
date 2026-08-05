//
// Copyright (c) 2017 CNRS
//
// SPDX-License-Identifier: BSD-2-Clause
//
// Derived from TSID (https://github.com/stack-of-tasks/tsid) and modified for
// cho_robot_project. Full license text: LICENSES/BSD-2-Clause-TSID.txt
//
#include <cho_controller_common/math/constraint_base.hpp>

namespace cho_controller {
namespace common {
namespace math {

ConstraintBase::ConstraintBase(const std::string & name):
  m_name(name){}

ConstraintBase::ConstraintBase(const std::string & name,
                               const unsigned int rows,
                               const unsigned int cols):
  m_name(name)
{
  m_A = Matrix::Zero(rows, cols);
}

ConstraintBase::ConstraintBase(const std::string & name,
                               ConstRefMatrix A):
  m_name(name),
  m_A(A)
{}

const std::string & ConstraintBase::name() const
{
  return m_name;
}

const Matrix & ConstraintBase::matrix() const
{
  return m_A;
}

Matrix & ConstraintBase::matrix()
{
  return m_A;
}

bool ConstraintBase::setMatrix(ConstRefMatrix A)
{
  assert(m_A.cols()==A.cols());
  assert(m_A.rows()==A.rows());
  m_A = A;
  return true;
}

} // namespace math
} // namespace common
} // namespace cho_controller