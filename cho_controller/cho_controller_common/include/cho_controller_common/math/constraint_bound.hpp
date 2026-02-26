#pragma once

#include "cho_controller_common/math/constraint_base.hpp"

namespace cho_controller {
namespace common {
namespace math {

class ConstraintBound : public ConstraintBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ConstraintBound(const std::string & name);

    ConstraintBound(const std::string & name,
                    const unsigned int size);

    ConstraintBound(const std::string & name,
                    ConstRefVector lb,
                    ConstRefVector ub);
    virtual ~ConstraintBound() {}

    unsigned int rows() const;
    unsigned int cols() const;
    void resize(const unsigned int r, const unsigned int c);

    bool isEquality() const;
    bool isInequality() const;
    bool isBound() const;

    const Vector & vector()     const;
    const Vector & lowerBound() const;
    const Vector & upperBound() const;

    Vector & vector();
    Vector & lowerBound();
    Vector & upperBound();

    bool setVector(ConstRefVector b);
    bool setLowerBound(ConstRefVector lb);
    bool setUpperBound(ConstRefVector ub);

    bool checkConstraint(ConstRefVector x, double tol=1e-6) const;

protected:
    Vector m_lb;
    Vector m_ub;
};

} // namespace math
} // namespace common
} // namespace cho_controller