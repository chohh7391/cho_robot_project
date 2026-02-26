#pragma once

#include "cho_controller_common/solver/fwd.hpp"
#include "cho_controller_common/solver/solver_HQP_output.hpp"
#include "cho_controller_common/math/constraint_base.hpp"

#include <vector>
#include <utility>

namespace cho_controller {
namespace common {
namespace solver {

class TSID_DLLAPI SolverHQPBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static std::string const HQP_status_string [5];

    typedef math::RefVector RefVector;
    typedef math::ConstRefVector ConstRefVector;
    typedef math::ConstRefMatrix ConstRefMatrix;

    SolverHQPBase(const std::string & name);
    virtual ~SolverHQPBase() {};

    virtual const std::string & name() { return m_name; }

    virtual void resize(unsigned int n, unsigned int neq, unsigned int nin) = 0;

    /** Solve the specified Hierarchical Quadratic Program.
     */
    virtual const HQPOutput & solve(const HQPData & problemData) = 0;

    virtual const HQPOutput & solve(const WHQPData & problemData) = 0;

    /** Get the objective value of the last solved problem. */
    virtual double getObjectiveValue() = 0;

    /** Return true if the solver is allowed to warm start, false otherwise. */
    virtual bool getUseWarmStart(){ return m_useWarmStart; }
    /** Specify whether the solver is allowed to use warm-start techniques. */
    virtual void setUseWarmStart(bool useWarmStart){ m_useWarmStart = useWarmStart; }

    /** Get the current maximum number of iterations performed by the solver. */
    virtual unsigned int getMaximumIterations(){ return m_maxIter; }
    /** Set the current maximum number of iterations performed by the solver. */
    virtual bool setMaximumIterations(unsigned int maxIter);


    /** Get the maximum time allowed to solve a problem. */
    virtual double getMaximumTime(){ return m_maxTime; }
    /** Set the maximum time allowed to solve a problem. */
    virtual bool setMaximumTime(double seconds);

protected:
    std::string           m_name;
    bool                  m_useWarmStart;   // true if the solver is allowed to warm start
    int                   m_maxIter;        // max number of iterations
    double                m_maxTime;        // max time to solve the HQP [s]
    HQPOutput             m_output;
};

} // namespace solver
} // namespace common
} // namespace cho_controller
