#include "cho_controller_common/tasks/task_se3_equality.hpp"
#include "cho_controller_common/robot/robot_wrapper.hpp"
#include "cho_controller_common/math/util.hpp"
#include <Eigen/SVD>

using namespace std;

namespace cho_controller {
namespace common {
namespace tasks {

using namespace math;
using namespace trajectory;
using namespace pinocchio;

TaskSE3Equality::TaskSE3Equality(
  const std::string & name, RobotWrapper & robot, const std::string & frameName, const Eigen::Vector3d & offset)
: TaskMotion(name, robot),
  m_frame_name(frameName),
  m_constraint(name, 6, robot.nv()),
  m_ref(12, 6),
  m_mobile(false),
  m_offset(offset)
{
  assert(m_robot.model().existFrame(frameName));
  m_frame_id = m_robot.model().getFrameId(frameName);

  m_v_ref.setZero();
  m_a_ref.setZero();
  m_M_ref.setIdentity();
  m_wMl.setIdentity();
  m_p_error_vec.setZero(6);
  m_v_error_vec.setZero(6);
  m_p.resize(12);
  m_v.resize(6);
  m_p_ref.resize(12);
  m_v_ref_vec.resize(6);
  m_Kp.setZero(6);
  m_Kd.setZero(6);
  m_a_des.setZero(6);
  
  m_J.setZero(6, robot.nv());
  m_J_rotated.setZero(6, robot.nv());

  if (m_mobile){
    m_constraint.resize(6, robot.nv()-3);
  }
  
  m_mask.resize(6);
  m_mask.fill(1.);
  setMask(m_mask);

  if (m_mobile)
    m_local_frame = false;
  else
    m_local_frame = true;

  m_wholebody = true;
}

void TaskSE3Equality::setMask(math::ConstRefVector mask)
{
  TaskMotion::setMask(mask);
  int n = dim();
  if (m_mobile){
    m_constraint.resize(n, (unsigned int)m_J.cols() - 3);
  }
  else{
    m_constraint.resize(n, (unsigned int)m_J.cols());
  }
  m_p_error_masked_vec.resize(n);
  m_v_error_masked_vec.resize(n);
  m_drift_masked.resize(n);
  m_a_des_masked.resize(n);
}

int TaskSE3Equality::dim() const
{
  return (int)m_mask.sum();
}

const Vector & TaskSE3Equality::Kp() const { return m_Kp; }

const Vector & TaskSE3Equality::Kd() const { return m_Kd; }

void TaskSE3Equality::Kp(ConstRefVector Kp)
{
  assert(Kp.size()==6);
  m_Kp = Kp;
}

void TaskSE3Equality::Kd(ConstRefVector Kd)
{
  assert(Kd.size()==6);
  m_Kd = Kd;
}

void TaskSE3Equality::setReference(TrajectorySample & ref)
{
  m_ref = ref;
  vectorToSE3(ref.pos, m_M_ref);
  m_v_ref = Motion(ref.vel);
  m_a_ref = Motion(ref.acc);
}

const TrajectorySample & TaskSE3Equality::getReference() const
{
  return m_ref;
}

const Vector & TaskSE3Equality::position_error() const
{
  return m_p_error_masked_vec;
}

const Vector & TaskSE3Equality::velocity_error() const
{
  return m_v_error_masked_vec;
}

const Vector & TaskSE3Equality::position() const
{
  return m_p;
}

const Vector & TaskSE3Equality::velocity() const
{
  return m_v;
}

const Vector & TaskSE3Equality::position_ref() const
{
  return m_p_ref;
}

const Vector & TaskSE3Equality::velocity_ref() const
{
  return m_v_ref_vec;
}

const Vector & TaskSE3Equality::getDesiredAcceleration() const
{
  return m_a_des_masked;
}

Vector TaskSE3Equality::getAcceleration(ConstRefVector dv) const
{
  return m_constraint.matrix()*dv + m_drift_masked;
}

Index TaskSE3Equality::frame_id() const
{
  return m_frame_id;
}

const ConstraintBase & TaskSE3Equality::getConstraint() const
{
  return m_constraint;
}

void TaskSE3Equality::useLocalFrame(bool local_frame)
{
  m_local_frame = local_frame;
}

const ConstraintBase & TaskSE3Equality::compute(
  const double , ConstRefVector ,ConstRefVector ,Data & data)
{
  SE3 oMi, oMi_prev;
  Motion v_frame;
  m_robot.framePosition(data, m_frame_id, oMi_prev);
  SE3 T_offset;
  T_offset.setIdentity();
  T_offset.translation(m_offset);
  
  using namespace std;
  oMi = oMi_prev * T_offset;


  m_robot.frameVelocity(data, m_frame_id, v_frame);
  Eigen::MatrixXd Adj_mat(6,6);
  Adj_mat.setIdentity();
  Eigen::Vector3d offset_local;
  offset_local = m_offset; //oMi.translation() - oMi_prev.translation();

  Adj_mat(0, 4) = -offset_local(2);
  Adj_mat(0, 5) = offset_local(1);
  Adj_mat(1, 3) = offset_local(2);
  Adj_mat(1, 5) = -offset_local(0);
  Adj_mat(2, 3) = -offset_local(1);
  Adj_mat(2, 4) = offset_local(0);

  // Adj_mat.topLeftCorner(3,3) = oMi.rotation();
  // Adj_mat.bottomRightCorner(3,3) = oMi.rotation();
  Adj_mat.topRightCorner(3,3) = -Adj_mat.topRightCorner(3,3);
  
  v_frame.linear() = v_frame.linear() + Adj_mat.topRightCorner(3,3) * v_frame.angular();
  m_robot.frameClassicAcceleration(data, m_frame_id, m_drift);
  m_drift.linear() = m_drift.linear() + Adj_mat.topRightCorner(3,3) * m_drift.angular();

  // @todo Since Jacobian computation is cheaper in world frame
  // we could do all computations in world frame
  
  m_robot.frameJacobianLocal(data, m_frame_id, m_J); // 6 by 9 (for husky with single franka arm)
  m_J = Adj_mat * m_J;

  m_M_ref = m_M_ref;

  // cout << m_M_ref << endl;
  errorInSE3(oMi, m_M_ref, m_p_error);          // pos err in local frame
  
  m_v_ref.linear() = m_v_ref.linear() + Adj_mat.topRightCorner(3,3) * m_v_ref.angular();
  m_a_ref.linear() = m_a_ref.linear() + Adj_mat.topRightCorner(3,3) * m_a_ref.angular();
  SE3ToVector(m_M_ref, m_p_ref);
  SE3ToVector(oMi, m_p);

  // Transformation from local to world
  m_wMl.rotation(oMi.rotation());

  if (m_local_frame) {
    m_p_error_vec = m_p_error.toVector();
    m_v_error =  m_wMl.actInv(m_v_ref) - v_frame;  // vel err in local frame

    // desired acc in local frame
    m_a_des = m_Kp.cwiseProduct(m_p_error_vec)
              + m_Kd.cwiseProduct(m_v_error.toVector())
              + m_wMl.actInv(m_a_ref).toVector();
  } else {
    m_p_error_vec = m_wMl.toActionMatrix() *   // pos err in local world-oriented frame
        m_p_error.toVector();

    m_v_error = m_v_ref - m_wMl.act(v_frame);  // vel err in local world-oriented frame

    m_drift = m_wMl.act(m_drift);

    // desired acc in local world-oriented frame
    m_a_des = m_Kp.cwiseProduct(m_p_error_vec)
              + m_Kd.cwiseProduct(m_v_error.toVector())
              + m_a_ref.toVector();

    // Use an explicit temporary `m_J_rotated` here to avoid allocations.
    m_J_rotated.noalias() = m_wMl.toActionMatrix() * m_J;
    m_J = m_J_rotated;
  }
  
  m_v_error_vec = m_v_error.toVector();
  m_v_ref_vec = m_v_ref.toVector();
  m_v = v_frame.toVector();

  if (!m_wholebody){
    m_J.topLeftCorner(6, 2).setZero();
  }
  int idx = 0;
  
  for (int i = 0; i < 6; i++) {
    if (m_mask(i) != 1.) continue;

    m_constraint.matrix().row(idx) = m_J.row(i);
    m_constraint.vector().row(idx) = (m_a_des - m_drift.toVector()).row(i);
    m_a_des_masked(idx)            = m_a_des(i);
    m_drift_masked(idx)            = m_drift.toVector()(i);
    m_p_error_masked_vec(idx)      = m_p_error_vec(i);
    m_v_error_masked_vec(idx)      = m_v_error_vec(i);

    idx += 1;
    
  }

  // using namespace Eigen;
  // if (m_mobile){
  //   JacobiSVD<Matrix> svd(m_constraint.matrix().block(0, 2, 6, 7), ComputeThinU | ComputeThinV);

  //   if (svd.singularValues()[m_constraint.vector().size() - 1] < 0.05){
  //     Matrix matrix_pre = m_constraint.matrix();
  //     Vector vector_pre = m_constraint.vector();

  //     Matrix matrix_new;
  //     Vector vector_new;

  //     matrix_new.setZero(matrix_pre.matrix().rows(), matrix_pre.matrix().cols());
  //     vector_new.setZero(matrix_pre.matrix().rows());

  //     matrix_new.topLeftCorner(matrix_pre.matrix().rows()-1, matrix_pre.matrix().cols()) = svd.matrixU().topLeftCorner(matrix_pre.matrix().rows(), matrix_pre.matrix().rows()-1).transpose() * matrix_pre;
  //     vector_new.head(matrix_pre.matrix().rows()-1) = svd.matrixU().topLeftCorner(matrix_pre.matrix().rows(), matrix_pre.matrix().rows()-1).transpose() * vector_pre;
    
  //     matrix_new.bottomLeftCorner(1, matrix_pre.matrix().cols()) = svd.matrixU().topRightCorner(matrix_pre.matrix().rows(), 1).transpose() * matrix_pre;
  //     vector_new.tail(1) = h_factor(svd.singularValues()[m_constraint.vector().size() - 1], 0.05, 0.02) * svd.matrixU().topRightCorner(matrix_pre.matrix().rows(), 1).transpose() * vector_pre;

  //     m_constraint.setMatrix(matrix_new);
  //     m_constraint.setVector(vector_new);
  //   }
  // }
  // else{
  //   JacobiSVD<Matrix> svd(m_constraint.matrix(), ComputeThinU | ComputeThinV);

  //   if (svd.singularValues()[m_constraint.vector().size() - 1] < 0.05){
  //     Matrix matrix_pre = m_constraint.matrix();
  //     Vector vector_pre = m_constraint.vector();

  //     Matrix matrix_new;
  //     Vector vector_new;

  //     matrix_new.setZero(matrix_pre.matrix().rows(), matrix_pre.matrix().cols());
  //     vector_new.setZero(matrix_pre.matrix().rows());

  //     matrix_new.topLeftCorner(matrix_pre.matrix().rows()-1, matrix_pre.matrix().cols()) = svd.matrixU().topLeftCorner(matrix_pre.matrix().rows(), matrix_pre.matrix().rows()-1).transpose() * matrix_pre;
  //     vector_new.head(matrix_pre.matrix().rows()-1) = svd.matrixU().topLeftCorner(matrix_pre.matrix().rows(), matrix_pre.matrix().rows()-1).transpose() * vector_pre;
    
  //     matrix_new.bottomLeftCorner(1, matrix_pre.matrix().cols()) = svd.matrixU().topRightCorner(matrix_pre.matrix().rows(), 1).transpose() * matrix_pre;
  //     vector_new.tail(1) = h_factor(svd.singularValues()[m_constraint.vector().size() - 1], 0.05, 0.02) * svd.matrixU().topRightCorner(matrix_pre.matrix().rows(), 1).transpose() * vector_pre;

  //     m_constraint.setMatrix(matrix_new);
  //     m_constraint.setVector(vector_new);
  //   }     
    
  // }
  
  return m_constraint;
}

} // namespace math
} // namespace common
} // namespace cho_controller