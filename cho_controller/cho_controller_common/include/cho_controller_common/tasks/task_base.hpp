#pragma once

#include "cho_controller_common/math/fwd.hpp"
#include "cho_controller_common/robot/robot_wrapper.hpp"
#include "cho_controller_common/math/constraint_base.hpp"

#include <pinocchio/multibody/fwd.hpp>


namespace cho_controller {
namespace common {
namespace tasks {

class TaskBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  typedef math::ConstraintBase ConstraintBase;
  typedef math::ConstRefVector ConstRefVector;
  typedef pinocchio::Data Data;
  typedef robot::RobotWrapper RobotWrapper;

  TaskBase(const std::string & name, RobotWrapper & robot);

  const std::string & name() const;

  void name(const std::string & name);
  
  /// \brief Return the dimension of the task.
  /// \info should be overloaded in the child class.
  virtual int dim() const = 0;

  virtual const ConstraintBase & compute(
    const double t,
    ConstRefVector q,
    ConstRefVector v,
    Data & data) = 0;

  virtual const ConstraintBase & getConstraint() const = 0;

protected:
  std::string m_name;
  
  /// \brief Reference on the robot model.
  robot::RobotWrapper & m_robot;
};

} // namespace tasks
} // namespace common
} // namespace cho_controller