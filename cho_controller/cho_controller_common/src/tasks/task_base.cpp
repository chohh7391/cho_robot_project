#include "cho_controller_common/tasks/task_base.hpp"

namespace cho_controller {
namespace common {
namespace tasks {

TaskBase::TaskBase(const std::string & name, RobotWrapper & robot)
: m_name(name), m_robot(robot) {}

const std::string & TaskBase::name() const
{
  return m_name;
}

void TaskBase::name(const std::string & name)
{
  m_name = name;
}

} // namespace math
} // namespace common
} // namespace cho_controller