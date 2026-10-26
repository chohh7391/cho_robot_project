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

#include <mujoco/mujoco.h>

#include <array>
#include <stdexcept>
#include <string>
#include <vector>

namespace cho_simulation_mujoco
{

// A MuJoCo body as the plugin reports it: its name and model index.
struct SceneBody
{
  std::string name;
  int id;
};

// The bodies named in `names`, in that order; with none named, every body whose
// first joint is a free joint -- the loose objects of a scene, not the robot's
// links. Throws std::invalid_argument naming a body the model does not have.
inline std::vector<SceneBody> select_bodies(
  const mjModel * model, const std::vector<std::string> & names)
{
  std::vector<SceneBody> bodies;
  if (names.empty()) {
    for (int id = 1; id < model->nbody; ++id) {
      if (model->body_jntnum[id] > 0 && model->jnt_type[model->body_jntadr[id]] == mjJNT_FREE) {
        const char * name = mj_id2name(model, mjOBJ_BODY, id);
        bodies.push_back({name ? name : "body_" + std::to_string(id), id});
      }
    }
    return bodies;
  }
  for (const auto & name : names) {
    const int id = mj_name2id(model, mjOBJ_BODY, name.c_str());
    if (id < 0) {
      throw std::invalid_argument("the MuJoCo model has no body '" + name + "'");
    }
    bodies.push_back({name, id});
  }
  return bodies;
}

// A body's pose in the MuJoCo world frame: position, then quaternion as x, y,
// z, w (ROS order; MuJoCo stores w first). Valid once mj_forward() or a step
// has run on `data`.
inline std::array<double, 7> world_pose(const mjData * data, int id)
{
  const mjtNum * p = data->xpos + 3 * id;
  const mjtNum * q = data->xquat + 4 * id;
  return {p[0], p[1], p[2], q[1], q[2], q[3], q[0]};
}

// When to publish next, on the simulation's own clock: every `period` seconds
// of simulated time, whatever the wall clock does. A first call publishes.
class PublishSchedule
{
public:
  explicit PublishSchedule(double period) : period_(period) {}

  // True when a sample is due at simulation time `t`; it then moves the next
  // one a period on from `t`, so a slow step does not cause a burst.
  bool due(double t)
  {
    if (t + 1e-12 < next_) {
      return false;
    }
    next_ = t + period_;
    return true;
  }

private:
  double period_;
  double next_{-1.0};
};

}  // namespace cho_simulation_mujoco
