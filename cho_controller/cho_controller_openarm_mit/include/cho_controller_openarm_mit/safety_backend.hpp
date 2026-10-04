// Copyright 2026 Hyunho Cho
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
