#include "cho_controller_common/solver/solver_HQP_qpoases.hpp"
#include "cho_controller_common/math/util.hpp"
#include "cho_controller_common/util/stop_watch.hpp"

namespace cho_controller {
namespace common {
namespace solver {

using namespace cho_controller::common::math;
using namespace cho_controller::common::solver;
using namespace Eigen;
using namespace qpOASES;
using namespace std;

SolverHQPoases::SolverHQPoases(const std::string & name):
  SolverHQPBase(name),
  m_hessian_regularization(DEFAULT_HESSIAN_REGULARIZATION)
{
  m_n = 0;
  m_neq = 0;
  m_nin = 0;

  m_options.setToDefault();
  m_options.initialStatusBounds = ST_INACTIVE;
  m_options.printLevel          = PL_HIGH; //PL_LOW
  m_options.enableRegularisation = BT_TRUE;
  m_options.enableEqualities = BT_TRUE;

}

void SolverHQPoases::sendMsg(const std::string & s)
{
  std::cout<<"[SolverHQPoases."<<m_name<<"] "<<s<<std::endl;
}

void SolverHQPoases::resize(unsigned int n, unsigned int neq, unsigned int nin)
{
  const bool resizeVar = n!=m_n;
  const bool resizeEq = (resizeVar || neq!=m_neq );
  const bool resizeIn = (resizeVar || nin!=m_nin );

  if(resizeEq)
  {
    #ifndef NDEBUG
    sendMsg("Resizing equality constraints from "+toString(m_neq)+" to "+toString(neq));
    #endif
    m_CE.resize(neq, n);
    m_ce0.resize(neq);
  }
  if(resizeIn)
  {
    #ifndef NDEBUG
    sendMsg("Resizing inequality constraints from "+toString(m_nin)+" to "+toString(nin));
    #endif
    m_CI.resize(2*nin, n);
    m_ci0.resize(2*nin);
  }
  if(resizeVar)
  {
    #ifndef NDEBUG
    sendMsg("Resizing Hessian from "+toString(m_n)+" to "+toString(n));
    #endif
    m_H.resize(n, n);
    m_g.resize(n);
    m_output.x.resize(n);
  }

  m_n = n;
  m_neq = neq;
  m_nin = nin;

  m_H_row.resize(n, n);
  m_A_row.resize(neq + 2*nin, n);
  m_Aub.resize(neq + 2*nin);
  m_Alb = m_Aub;

  m_solver = SQProblem(n, neq + nin*2);
  m_solver.setOptions(m_options);
}

const HQPOutput & SolverHQPoases::solve(const HQPData & problemData)
{
  if(problemData.size()>2)
  {
    assert(false && "Solver not implemented for more than 2 hierarchical levels.");
  }


  unsigned int neq = 0, nin = 0;
  const ConstraintLevel & cl0 = problemData[0];
  if(cl0.size()>0)
  {
    const unsigned int n = cl0[0].second->cols();
    for(ConstraintLevel::const_iterator it=cl0.begin(); it!=cl0.end(); it++)
    {
      auto constr = it->second;
      assert(n==constr->cols());
      if(constr->isEquality())
        neq += constr->rows();
      else
        nin += constr->rows();
    }
    // If necessary, resize the constraint matrices
    resize(n, neq, nin);

    int i_eq=0, i_in=0;
    for(ConstraintLevel::const_iterator it=cl0.begin(); it!=cl0.end(); it++)
    {
      auto constr = it->second;
      if(constr->isEquality())
      {
        m_CE.middleRows(i_eq, constr->rows()) = constr->matrix();
        m_ce0.segment(i_eq, constr->rows())   = -constr->vector();
        i_eq += constr->rows();
      }
      else if(constr->isInequality())
      {
        m_CI.middleRows(i_in, constr->rows()) = constr->matrix();
        m_ci0.segment(i_in, constr->rows())   = -constr->lowerBound();
        i_in += constr->rows();
        m_CI.middleRows(i_in, constr->rows()) = -constr->matrix();
        m_ci0.segment(i_in, constr->rows())   = constr->upperBound();
        i_in += constr->rows();
      }
      else if(constr->isBound())
      {
        m_CI.middleRows(i_in, constr->rows()).setIdentity();
        m_ci0.segment(i_in, constr->rows())   = -constr->lowerBound();
        i_in += constr->rows();
        m_CI.middleRows(i_in, constr->rows()) = -Matrix::Identity(m_n, m_n);
        m_ci0.segment(i_in, constr->rows())   = constr->upperBound();
        i_in += constr->rows();
      }
    }
  }
  else
    resize(m_n, neq, nin);

  if(problemData.size()>1)
  {
    const ConstraintLevel & cl1 = problemData[1];
    m_H.setZero();
    m_g.setZero();
    for(ConstraintLevel::const_iterator it=cl1.begin(); it!=cl1.end(); it++)
    {
      const double & w = it->first;
      auto constr = it->second;
      if(!constr->isEquality())
        assert(false && "Inequalities in the cost function are not implemented yet");

      m_H += w*constr->matrix().transpose()*constr->matrix();
      m_g -= w*(constr->matrix().transpose()*constr->vector());
    }
    m_H.diagonal() += m_hessian_regularization*Vector::Ones(m_n);
  }

  // eiquadprog
  //  min 0.5 * x G x + g0 x
  //  s.t.
  //  CE^T x + ce0 = 0
  //  CI^T x + ci0 >= 0
  
  // qpoases
  // min 0.5 *  x H x + x^t * g
  // s.t lbA < Ax < ubA
  int iter = 1000;
  m_H_row = m_H;
  m_A_row.topRows(m_CE.rows()) = m_CE;
  m_Alb.topRows(m_ce0.rows()) = -m_ce0;
  m_Aub.topRows(m_ce0.rows()) = -m_ce0;

  m_A_row.bottomRows(m_CI.rows()) = m_CI;
  m_Alb.bottomRows(m_ci0.rows()) = -m_ci0;
  m_Aub.bottomRows(m_ci0.rows()) = 10000.0 *Vector::Ones(m_ci0.rows());

  // cout << "m_H" << m_H_row << endl;
  // cout << "m_A" << m_A_row << endl;
  // cout << "m_Alb" << m_Alb.transpose() << endl;
  // cout << "m_Aub" << m_Aub.transpose() << endl;
  // cout << "m_CI" << m_CI << endl; 
  
  m_status = m_solver.init(m_H_row.data(), m_g.data(), m_A_row.transpose().data(), 0, 0, m_Alb.data(), m_Aub.data(), iter);
  m_solver.getPrimalSolution(m_output.x.data());
  // cout << m_output.x.transpose() << "result" << endl;
  // getchar();

  if(m_status != SUCCESSFUL_RETURN)
    m_output.status = HQP_STATUS_INFEASIBLE;
  else
  {
    m_output.status = HQP_STATUS_OPTIMAL;
    m_solver.getPrimalSolution(m_output.x.data());
  }

  return m_output;
}

double SolverHQPoases::getObjectiveValue()
{
  return m_objValue;
}

} // namespace solver
} // namespace common
} // namespace cho_controller