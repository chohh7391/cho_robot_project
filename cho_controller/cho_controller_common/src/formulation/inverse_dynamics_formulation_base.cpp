#include "cho_controller_common/formulation/inverse_dynamics_formulation_base.hpp"

namespace cho_controller {
namespace common {
namespace formulation {

TaskLevel::TaskLevel(tasks::TaskBase & t, unsigned int priority)
: task(t), priority(priority) {}

TaskLevelForce::TaskLevelForce(tasks::TaskContactForce & task, unsigned int priority)
:task(task), priority(priority) {}
  
InverseDynamicsFormulationBase::InverseDynamicsFormulationBase(
  const std::string & name, RobotWrapper & robot, bool verbose)
: m_name(name), m_robot(robot), m_verbose(verbose) {}

// bool InverseDynamicsFormulationBase::addRigidContact(ContactBase & contact)
// {
//   return addRigidContact(contact, 1e-5);
// }

} // namespace formulation
} // namespace common
} // namespace cho_controller