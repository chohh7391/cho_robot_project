#pragma once

#include "cho_controller_common/math/fwd.hpp"
#include "cho_controller_common/solver/fwd.hpp"
#include "cho_controller_common/solver/solver_HQP_base.hpp"

#include <eiquadprog/eiquadprog-rt.hpp>

namespace cho_controller {
namespace common {
namespace solver {

template<int nVars, int nEqCon, int nIneqCon>
class TSID_DLLAPI SolverHQuadProgRT : public SolverHQPBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef math::Matrix Matrix;
    typedef math::Vector Vector;
    typedef math::RefVector RefVector;
    typedef math::ConstRefVector ConstRefVector;
    typedef math::ConstRefMatrix ConstRefMatrix;

    SolverHQuadProgRT(const std::string & name);

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

    eiquadprog::solvers::RtEiquadprog<nVars, nEqCon, 2*nIneqCon> m_solver;

    typename RtMatrixX<nVars, nVars>::d m_H;
    typename RtVectorX<nVars>::d m_g;
    typename RtMatrixX<nEqCon, nVars>::d m_CE;
    typename RtVectorX<nEqCon>::d m_ce0;
    typename RtMatrixX<2*nIneqCon, nVars>::d m_CI;  /// twice the rows because inequality constraints are bilateral
    typename RtVectorX<2*nIneqCon>::d m_ci0;
    double m_objValue;

    double m_hessian_regularization;

    Eigen::VectorXi m_activeSet;  /// vector containing the indexes of the active inequalities
    int m_activeSetSize;

    //      Eigen::FullPivLU<template RtMatrixX<nEqCon, nVars> > m_CE_lu;
    // ColPivHouseholderQR

    int m_neq;  /// number of equality constraints
    int m_nin;  /// number of inequality constraints
    int m_n;    /// number of variables
};

} // namespace solver
} // namespace common
} // namespace cho_controller
