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

// The one HeldCommandLedger of the process. Compiled into its own shared
// library, not left inline in the header: every controller plugin is a
// separately dlopen()ed library, and a header-only static would give each of
// them a private copy, so no controller would see what another released.
#include <mutex>
#include <string>
#include <unordered_map>

#include "cho_controller_base/held_command.hpp"

namespace cho_controller_base
{
namespace
{
struct Records
{
  std::mutex mutex;
  std::unordered_map<std::string, HeldCommandLedger::Release> by_interface;
};

Records & records()
{
  static Records instance;
  return instance;
}
}  // namespace

void HeldCommandLedger::record(const std::string & interface_name, double command, double measured)
{
  auto & all = records();
  std::lock_guard<std::mutex> lock(all.mutex);
  all.by_interface[interface_name] = Release{command, measured};
}

bool HeldCommandLedger::find(const std::string & interface_name, Release & out)
{
  auto & all = records();
  std::lock_guard<std::mutex> lock(all.mutex);
  const auto it = all.by_interface.find(interface_name);
  if (it == all.by_interface.end()) {
    return false;
  }
  out = it->second;
  return true;
}

void HeldCommandLedger::clear()
{
  auto & all = records();
  std::lock_guard<std::mutex> lock(all.mutex);
  all.by_interface.clear();
}

}  // namespace cho_controller_base
