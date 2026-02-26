#pragma once

#include "cho_controller_common/math/fwd.hpp"
#include <string>

namespace cho_controller {
namespace common {
namespace math {

class ConstraintBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ConstraintBase(const std::string & name);

    ConstraintBase(const std::string & name,
                    const unsigned int rows,
                    const unsigned int cols);

    ConstraintBase(const std::string & name,
                    ConstRefMatrix A);
    virtual ~ConstraintBase() {}

    virtual const std::string & name() const;
    virtual unsigned int rows() const = 0;
    virtual unsigned int cols() const = 0;
    virtual void resize(const unsigned int r, const unsigned int c) = 0;

    virtual bool isEquality() const = 0;
    virtual bool isInequality() const = 0;
    virtual bool isBound() const = 0;

    virtual const Matrix & matrix() const;
    virtual const Vector & vector() const = 0;
    virtual const Vector & lowerBound() const = 0;
    virtual const Vector & upperBound() const = 0;

    virtual Matrix & matrix();
    virtual Vector & vector() = 0;
    virtual Vector & lowerBound() = 0;
    virtual Vector & upperBound() = 0;

    virtual bool setMatrix(ConstRefMatrix A);
    virtual bool setVector(ConstRefVector b) = 0;
    virtual bool setLowerBound(ConstRefVector lb) = 0;
    virtual bool setUpperBound(ConstRefVector ub) = 0;

    virtual bool checkConstraint(ConstRefVector x, double tol=1e-6) const = 0;

protected:            
    std::string m_name;
    Matrix m_A;
};


} // namespace math
} // namespace common
} // namespace cho_controller
