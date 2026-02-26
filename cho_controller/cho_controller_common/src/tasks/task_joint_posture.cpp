#include "cho_controller_common/tasks/task_joint_posture.hpp"
#include "cho_controller_common/robot/robot_wrapper.hpp"

namespace cho_controller {
namespace common {
namespace tasks {

using namespace math;
using namespace trajectory;
using namespace pinocchio;

TaskJointPosture::TaskJointPosture(
  const std::string & name, RobotWrapper & robot)
: TaskMotion(name, robot), m_constraint(name, robot.na(), robot.nv()), m_ref(robot.na()), m_mobile(false)
{
  m_Kp.setZero(robot.na());
  m_Kd.setZero(robot.na());
  Vector m = Vector::Ones(robot.na());
  setMask(m);        
}

const Vector & TaskJointPosture::mask() const
{
  return m_mask;
}

void TaskJointPosture::mask(const Vector & m)
{
  return setMask(m);
}

void TaskJointPosture::setMask(ConstRefVector m)
{
  if (!m_mobile){
    assert(m.size()==m_robot.na());
    m_mask = m;
    const Vector::Index dim = static_cast<Vector::Index>(m.sum());
    Matrix S = Matrix::Zero(dim, m_robot.nv());
    m_activeAxes.resize(dim);
    unsigned int j=0;
    for(unsigned int i=0; i<m.size(); i++)
      if(m(i)!=0.0)
      {
        assert(m(i)==1.0);
        S(j,m_robot.nv()-m_robot.na()+i) = 1.0;
        m_activeAxes(j) = i;
        j++;
      }
    m_constraint.resize((unsigned int)dim, m_robot.nv());
    m_constraint.setMatrix(S);
  }
  else{
    assert(m.size()==m_robot.na()-2);
    m_mask = m;
    const Vector::Index dim = static_cast<Vector::Index>(m.sum());
    Matrix S = Matrix::Zero(dim, m_robot.na());
    m_activeAxes.resize(dim);
    unsigned int j=0;
    for(unsigned int i=0; i<m.size(); i++)
      if(m(i)!=0.0)
      {
        assert(m(i)==1.0);
        S(j, 2+i) = 1.0;
        m_activeAxes(j) = i;
        j++;
      }
    m_constraint.resize((unsigned int)dim, m_robot.na());
    m_constraint.setMatrix(S);
  }
}

int TaskJointPosture::dim() const
{
  return (int)m_mask.sum();
}

const Vector & TaskJointPosture::Kp(){ return m_Kp; }

const Vector & TaskJointPosture::Kd(){ return m_Kd; }

void TaskJointPosture::Kp(ConstRefVector Kp)
{
  if (!m_mobile){
    assert(Kp.size()==m_robot.na());
  }
  else{
    assert(Kp.size()==m_robot.na()-2);
  }
  m_Kp = Kp;
}

void TaskJointPosture::Kd(ConstRefVector Kd)
{
  if (!m_mobile){
    assert(Kd.size()==m_robot.na());
  }
  else{
    assert(Kd.size()==m_robot.na()-2);
  }
  m_Kd = Kd;
}

void TaskJointPosture::setReference(const TrajectorySample & ref)
{
  if (!m_mobile){
    assert(ref.pos.size()==m_robot.na());
    assert(ref.vel.size()==m_robot.na());
    assert(ref.acc.size()==m_robot.na());
  }
  else{
    assert(ref.pos.size()==m_robot.na()-2);
    assert(ref.vel.size()==m_robot.na()-2);
    assert(ref.acc.size()==m_robot.na()-2);
  }
  m_ref = ref;
}

const TrajectorySample & TaskJointPosture::getReference() const
{
  return m_ref;
}

const Vector & TaskJointPosture::getDesiredAcceleration() const
{
  return m_a_des;
}

Vector TaskJointPosture::getAcceleration(ConstRefVector dv) const
{
  return m_constraint.matrix()*dv;
}

const Vector & TaskJointPosture::position_error() const
{
  return m_p_error;
}

const Vector & TaskJointPosture::velocity_error() const
{
  return m_v_error;
}

const Vector & TaskJointPosture::position() const
{
  return m_p;
}

const Vector & TaskJointPosture::velocity() const
{
  return m_v;
}

const Vector & TaskJointPosture::position_ref() const
{
  return m_ref.pos;
}

const Vector & TaskJointPosture::velocity_ref() const
{
  return m_ref.vel;
}

const ConstraintBase & TaskJointPosture::getConstraint() const
{
  return m_constraint;
}

const ConstraintBase & TaskJointPosture::compute(const double ,
                                                ConstRefVector q,
                                                ConstRefVector v,
                                                Data & )
{
  if (!m_mobile){
    // Compute errors
    m_p = q.tail(m_robot.na());
    m_v = v.tail(m_robot.na());
    m_p_error = m_p - m_ref.pos;
    m_v_error = m_v - m_ref.vel;
    m_a_des = - m_Kp.cwiseProduct(m_p_error)
              - m_Kd.cwiseProduct(m_v_error)
              + m_ref.acc;
    
    for(unsigned int i=0; i<m_activeAxes.size(); i++)
      m_constraint.vector()(i) = m_a_des(m_activeAxes(i));
    return m_constraint;
  }
  else{
    m_p = q.tail(m_robot.na()-2);
    m_v = v.tail(m_robot.na()-2);
    m_p_error = m_p - m_ref.pos;
    m_v_error = m_v - m_ref.vel;
    m_a_des = - m_Kp.cwiseProduct(m_p_error)
              - m_Kd.cwiseProduct(m_v_error)
              + m_ref.acc;
              
    for(unsigned int i=0; i<m_activeAxes.size(); i++)
      m_constraint.vector()(i) = m_a_des(m_activeAxes(i));
    return m_constraint;
  }
}

} // namespace tasks
} // namespace common
} // namespace cho_controller