#pragma once

#include "cho_controller_common/solver/solver_HQP_base.hpp"
#include <eiquadprog/eiquadprog-fast.hpp>

namespace cho_controller {
namespace common {
namespace solver {

class TSID_DLLAPI SolverHQuadProgFast : public SolverHQPBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef math::Matrix Matrix;
    typedef math::Vector Vector;
    typedef math::RefVector RefVector;
    typedef math::ConstRefVector ConstRefVector;
    typedef math::ConstRefMatrix ConstRefMatrix;

    SolverHQuadProgFast(const std::string & name);

    void resize(unsigned int n, unsigned int neq, unsigned int nin);

    /** Solve the given Hierarchical Quadratic Program
     */
    const HQPOutput & solve(const HQPData & problemData);
    const HQPOutput & solve(const WHQPData & problemData){
        return m_output;
    };

    /** Get the objective value of the last solved problem. */
    double getObjectiveValue();

    /** Set the current maximum number of iterations performed by the solver. */
    bool setMaximumIterations(unsigned int maxIter);

protected:

    void sendMsg(const std::string & s);

    // <nVars, nEqCon, 2*nIneqCon>
    eiquadprog::solvers::EiquadprogFast m_solver; 

    Matrix m_H;
    Vector m_g;
    Matrix m_CE;
    Vector m_ce0;
    Matrix m_CI;  /// twice the rows because inequality constraints are bilateral
    Vector m_ci0;
    double m_objValue;

    double m_hessian_regularization;

    Eigen::VectorXi m_activeSet;  /// vector containing the indexes of the active inequalities
    int m_activeSetSize;

    unsigned int m_neq;  /// number of equality constraints
    unsigned int m_nin;  /// number of inequality constraints
    unsigned int m_n;    /// number of variables
};

} // namespace solver
} // namespace common
} // namespace cho_controller