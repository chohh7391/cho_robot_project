#include <cho_controller_common/math/constraint_equality.hpp>


namespace cho_controller {
namespace common {
namespace math {

ConstraintEquality::ConstraintEquality(const std::string & name):
  ConstraintBase(name)
{}

ConstraintEquality::ConstraintEquality(const std::string & name,
                                       const unsigned int rows,
                                       const unsigned int cols):
  ConstraintBase(name, rows, cols),
  m_b(Vector::Zero(rows))
{}

ConstraintEquality::ConstraintEquality(const std::string & name,
                                       ConstRefMatrix A,
                                       ConstRefVector b):
  ConstraintBase(name, A),
  m_b(b)
{
  
}

unsigned int ConstraintEquality::rows() const
{
  assert(m_A.rows()==m_b.rows());
  return (unsigned int) m_A.rows();
}

unsigned int ConstraintEquality::cols() const
{
  return (unsigned int) m_A.cols();
}

void ConstraintEquality::resize(const unsigned int r, const unsigned int c)
{
  m_A.setZero(r, c);
  m_b.setZero(r);
}

bool ConstraintEquality::isEquality() const    { return true; }
bool ConstraintEquality::isInequality() const  { return false; }
bool ConstraintEquality::isBound() const       { return false; }

const Vector & ConstraintEquality::vector()     const { return m_b; }
const Vector & ConstraintEquality::lowerBound() const { assert(false); return m_b; }
const Vector & ConstraintEquality::upperBound() const { assert(false); return m_b; }

Vector & ConstraintEquality::vector()     { return m_b; }
Vector & ConstraintEquality::lowerBound() { assert(false); return m_b; }
Vector & ConstraintEquality::upperBound() { assert(false); return m_b;}

bool ConstraintEquality::setVector(ConstRefVector b) { m_b = b; return true; }
bool ConstraintEquality::setLowerBound(ConstRefVector ) { assert(false); return false; }
bool ConstraintEquality::setUpperBound(ConstRefVector ) { assert(false); return false; }

bool ConstraintEquality::checkConstraint(ConstRefVector x, double tol) const
{
  return (m_A*x-m_b).norm() < tol;
}

} // namespace math
} // namespace common
} // namespace cho_controller