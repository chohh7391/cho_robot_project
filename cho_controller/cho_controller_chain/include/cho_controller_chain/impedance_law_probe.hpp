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

#include <cstdint>
#include <vector>

namespace cho_controller_chain
{

// Test seam, read between controller_manager cycles (the harness runs update on the test's
// thread). Header-only, so a test casts the controller pluginlib loaded to it
// (dynamic_cast<ImpedanceLawProbe *>(controller.get())) without linking the plugin library: two
// copies of that library in one process confuse class_loader.
struct ImpedanceLawProbe
{
  virtual ~ImpedanceLawProbe() = default;
  // on_activate calls since construction; a restart by the controller_manager counts one.
  virtual std::uint64_t activations() const = 0;
  // reference_interfaces_: all positions, then all velocities, then all accelerations, in `joints`
  // order (3 per joint). NaN after configure, until the first activation.
  virtual const std::vector<double> & references() const = 0;
  // The reference used when not chained: the pose (positions, one per joint) measured at the last
  // on_activate, with zero velocity and acceleration. NaN until the first activation.
  virtual const std::vector<double> & hold() const = 0;
};

}  // namespace cho_controller_chain
