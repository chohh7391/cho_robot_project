#pragma once

#include "cho_controller_common/tasks/task_base.hpp"
#include "cho_controller_common/formulation/contact_level.hpp"
#include <memory>

namespace cho_controller {
namespace common {
namespace tasks {

using namespace cho_controller::common::formulation;

class TaskContactForce : public TaskBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  TaskContactForce(const std::string & name, RobotWrapper & robot);

  /**
   * Contact force tasks have an additional compute method that takes as extra input
   * argument the list of active contacts. This can be needed for force tasks that
   * involve all contacts, such as the CoP task.
   */
  virtual const ConstraintBase & compute(const double t,
                                          ConstRefVector q,
                                          ConstRefVector v,
                                          Data & data,
                                          const std::vector<std::shared_ptr<ContactLevel> >  *contacts) = 0;

  /**
   * Return the name of the contact associated to this task if this task is associated to a specific contact.
   * If this task is associated to multiple contact forces (all of them), returns an empty string.
   */
  virtual const std::string& getAssociatedContactName() = 0;
};

} // namespace tasks
} // namespace common
} // namespace cho_controller