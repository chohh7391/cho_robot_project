#include "cho_controller_common/formulation/contact_level.hpp"

namespace cho_controller {
namespace common {
namespace formulation {

ContactLevel::ContactLevel(cho_controller::common::contact::ContactBase & contact)
: contact(contact) {}

} // namespace formulation
} // namespace common
} // namespace cho_controller