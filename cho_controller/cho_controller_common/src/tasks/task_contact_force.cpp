#include "cho_controller_common/tasks/task_contact_force.hpp"

namespace cho_controller {
namespace common {
namespace tasks {

TaskContactForce::TaskContactForce(const std::string & name, cho_controller::common::robot::RobotWrapper & robot)
: TaskBase(name, robot) {}

} // namespace tasks
} // namespace common
} // namespace cho_controller