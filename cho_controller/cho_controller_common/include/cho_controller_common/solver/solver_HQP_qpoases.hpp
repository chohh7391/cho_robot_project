

#include "cho_controller_common/solver/solver_HQP_base.hpp"
#include <qpOASES.hpp>

namespace cho_controller {
namespace common {
namespace solver {

using namespace qpOASES;
typedef Eigen::Matrix<double, -1, -1, Eigen::RowMajor> Matrix_Row;

class TSID_DLLAPI SolverHQPoases: public SolverHQPBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef math::Matrix Matrix;
    typedef math::Vector Vector;
    typedef math::RefVector RefVector;
    typedef math::ConstRefVector ConstRefVector;
    typedef math::ConstRefMatrix ConstRefMatrix;

    SolverHQPoases(const std::string & name);

    void resize(unsigned int n, unsigned int neq, unsigned int nin);

    /** Solve the given Hierarchical Quadratic Program
     */
    const HQPOutput & solve(const HQPData & problemData);
    const HQPOutput & solve(const WHQPData & problemData){
        return m_output;
    };
    /** Get the objective value of the last solved problem. */
    double getObjectiveValue();

protected:

    void sendMsg(const std::string & s);

    Matrix m_H;
    Vector m_g;
    Matrix m_CE;
    Vector m_ce0;
    Matrix m_CI;
    Vector m_ci0;
    double m_objValue;
    Matrix_Row m_H_row, m_A_row;
    Vector m_Aub, m_Alb;

    double m_hessian_regularization;

    Eigen::VectorXi m_activeSet;  /// vector containing the indexes of the active inequalities
    cho_controller::common::math::Index m_activeSetSize;

    SQProblem m_solver;
    Options m_options;
    returnValue m_status;

#ifdef ELIMINATE_EQUALITY_CONSTRAINTS
//      Eigen::FullPivLU<Matrix>                        m_CE_dec;
//	  Eigen::ColPivHouseholderQR<Matrix>              m_CE_dec; // fast, but difficult to retrieve null space basis
//      Eigen::FullPivHouseholderQR<Matrix>             m_CE_dec; // doc says it is slow 
    Eigen::CompleteOrthogonalDecomposition<Matrix>  m_CE_dec; // available from Eigen 3.3.0, 40 us for decomposition, 40 us to get null space basis, 40 us to project Hessian
//      Eigen::JacobiSVD<Matrix, Eigen::HouseholderQRPreconditioner> m_CE_dec; // too slow
    Matrix m_ZT_H_Z;
    Matrix m_CI_Z;
#endif

    unsigned int m_neq;  /// number of equality constraints
    unsigned int m_nin;  /// number of inequality constraints
    unsigned int m_n;    /// number of variables
};

} // namespace solver
} // namespace common
} // namespace cho_controller