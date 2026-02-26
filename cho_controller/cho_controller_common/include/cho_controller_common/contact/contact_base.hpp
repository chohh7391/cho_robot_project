#pragma once

#include "cho_controller_common/math/fwd.hpp"
#include "cho_controller_common/robot/fwd.hpp"
#include "cho_controller_common/tasks/task_se3_equality.hpp"

namespace cho_controller {
namespace common {
namespace contact {

///
/// \brief Base template of a Contact.
///
class ContactBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  typedef math::ConstraintBase ConstraintBase;
  typedef math::ConstraintInequality ConstraintInequality;
  typedef math::ConstraintEquality ConstraintEquality;
  typedef math::ConstRefVector ConstRefVector;
  typedef math::Matrix Matrix;
  typedef math::Matrix3x Matrix3x;
  typedef tasks::TaskSE3Equality TaskSE3Equality;
  typedef pinocchio::Data Data;
  typedef robot::RobotWrapper RobotWrapper;

  ContactBase(const std::string & name, RobotWrapper & robot);

  const std::string & name() const;

  void name(const std::string & name);
  
  /// Return the number of motion constraints
  virtual unsigned int n_motion() const = 0;

  /// Return the number of force variables
  virtual unsigned int n_force() const = 0;

  virtual const ConstraintBase & computeMotionTask(const double t,
                                                    ConstRefVector q,
                                                    ConstRefVector v,
                                                    Data & data) = 0;

  virtual const ConstraintInequality & computeForceTask(const double t,
                                                        ConstRefVector q,
                                                        ConstRefVector v,
                                                        const Data & data) = 0;

  virtual const Matrix & getForceGeneratorMatrix() = 0;

  virtual const ConstraintEquality & computeForceRegularizationTask(const double t,
                                                                    ConstRefVector q,
                                                                    ConstRefVector v,
                                                                    const Data & data) = 0;

  virtual const TaskSE3Equality & getMotionTask() const = 0;
  virtual const ConstraintBase & getMotionConstraint() const = 0;
  virtual const ConstraintInequality & getForceConstraint() const = 0;
  virtual const ConstraintEquality & getForceRegularizationTask() const = 0;
  
  virtual double getMinNormalForce() const = 0;
  virtual double getMaxNormalForce() const = 0;
  virtual bool setMinNormalForce(const double minNormalForce) = 0;
  virtual bool setMaxNormalForce(const double maxNormalForce) = 0;
  virtual double getNormalForce(ConstRefVector f) const = 0;
  virtual const Matrix3x & getContactPoints() const = 0;

protected:
  std::string m_name;
  /// \brief Reference on the robot model.
  RobotWrapper & m_robot;
};

} // namespace contact
} // namespace common
} // namespace cho_controller
