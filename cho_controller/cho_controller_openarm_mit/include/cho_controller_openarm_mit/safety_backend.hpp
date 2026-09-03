#pragma once

#include <stdexcept>
#include <string>

#include "cho_openarm_mit_core/mit_protocol.hpp"

namespace cho_controller_openarm_mit
{

// Controller configuration deliberately does not infer the backend from a
// profile name.  The selected backend is part of the safety contract passed to
// the profile loader, which rejects mismatched profiles before activation.
inline cho_openarm_mit_core::SafetyBackend safety_backend_from_parameter(
  const std::string & value)
{
  if (value == "mujoco") return cho_openarm_mit_core::SafetyBackend::MUJOCO;
  if (value == "real") return cho_openarm_mit_core::SafetyBackend::REAL;
  throw std::invalid_argument("safety_backend must be exactly 'mujoco' or 'real'");
}

}  // namespace cho_controller_openarm_mit
