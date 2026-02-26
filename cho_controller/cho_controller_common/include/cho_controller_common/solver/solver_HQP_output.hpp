#pragma once

#include "cho_controller_common/solver/fwd.hpp"
#include "cho_controller_common/math/fwd.hpp"

#include <vector>

namespace cho_controller {
namespace common {
namespace solver {

class HQPOutput
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef math::Vector Vector;
    typedef math::VectorXi VectorXi;

    HQPStatus status;    /// solver status
    Vector x;            /// solution
    Vector lambda;       /// Lagrange multipliers
    VectorXi activeSet;  /// indexes of active inequalities
    int iterations;      /// number of iterations performed by the solver

    HQPOutput(){}

    HQPOutput(int nVars, int nEqCon, int nInCon)
    {
        resize(nVars, nEqCon, nInCon);
    }

    void resize(int nVars, int nEqCon, int nInCon)
    {
        x.resize(nVars);
        lambda.resize(nEqCon+nInCon);
        activeSet.resize(nInCon);
    }
};

} // namespace solver
} // namespace common
} // namespace cho_controller
