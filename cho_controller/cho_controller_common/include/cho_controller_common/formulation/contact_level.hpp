#pragma once

#include "cho_controller_common/math/fwd.hpp"
#include "cho_controller_common/contact/contact_base.hpp"

namespace cho_controller {
namespace common {
namespace formulation {

using namespace cho_controller::common::contact;

/** Data structure collecting information regarding a single contact.
 * In particular, this structure contains the index of the force corresponding
 * to this contact in the force vector used as decision variable in the QP.
 * Moreover it contains all the default constraints associated to a contact for representing
 * the motion constraints (contact points do not move), the friction cone constraints
 * and the force regularization cost.
 */
struct ContactLevel
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  contact::ContactBase & contact;
  std::shared_ptr<math::ConstraintBase> motionConstraint;
  std::shared_ptr<math::ConstraintInequality> forceConstraint;
  std::shared_ptr<math::ConstraintEquality> forceRegTask;
  unsigned int index; /// index of 1st element of associated force variable in the force vector

  ContactLevel(contact::ContactBase & cons);
};

} // namespace formulation
} // namespace common
} // namespace cho_controller