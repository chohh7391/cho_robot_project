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

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "cho_simulation_mujoco/scene_bodies.hpp"

namespace
{

using cho_simulation_mujoco::PublishSchedule;
using cho_simulation_mujoco::select_bodies;
using cho_simulation_mujoco::world_pose;

// A one-link arm and two loose objects, the second turned 90 deg about z.
constexpr char kScene[] = R"(<mujoco>
  <worldbody>
    <body name="link1" pos="0 0 0.1">
      <joint name="joint1" axis="0 0 1"/>
      <geom type="capsule" size="0.02 0.1"/>
    </body>
    <body name="box" pos="0.5 0 0.025">
      <freejoint/>
      <geom type="box" size="0.025 0.025 0.025"/>
    </body>
    <body name="cup" pos="0.4 0.2 0.05" quat="0.7071068 0 0 0.7071068">
      <freejoint/>
      <geom type="cylinder" size="0.03 0.05"/>
    </body>
  </worldbody>
</mujoco>)";

struct Scene
{
  Scene()
  {
    char error[512] = "";
    mjSpec * spec = mj_parseXMLString(kScene, nullptr, error, sizeof(error));
    EXPECT_NE(spec, nullptr) << error;
    model = mj_compile(spec, nullptr);
    mj_deleteSpec(spec);
    data = mj_makeData(model);
    mj_forward(model, data);
  }
  ~Scene()
  {
    mj_deleteData(data);
    mj_deleteModel(model);
  }
  mjModel * model{nullptr};
  mjData * data{nullptr};
};

std::vector<std::string> names(const std::vector<cho_simulation_mujoco::SceneBody> & bodies)
{
  std::vector<std::string> out;
  for (const auto & body : bodies) {
    out.push_back(body.name);
  }
  return out;
}

TEST(SceneBodies, WithNoneNamedEveryFreeBodyIsReportedAndNoRobotLink)
{
  Scene scene;
  EXPECT_EQ(names(select_bodies(scene.model, {})), (std::vector<std::string>{"box", "cup"}));
}

TEST(SceneBodies, NamedBodiesComeInTheGivenOrderRobotLinksIncluded)
{
  Scene scene;
  EXPECT_EQ(
    names(select_bodies(scene.model, {"cup", "link1"})),
    (std::vector<std::string>{"cup", "link1"}));
  EXPECT_THROW(select_bodies(scene.model, {"bottle"}), std::invalid_argument);
}

TEST(SceneBodies, ThePoseIsInTheWorldFrameWithTheQuaternionInRosOrder)
{
  Scene scene;
  const auto cup = select_bodies(scene.model, {"cup"}).front();
  const auto pose = world_pose(scene.data, cup.id);
  EXPECT_NEAR(pose[0], 0.4, 1e-9);
  EXPECT_NEAR(pose[1], 0.2, 1e-9);
  EXPECT_NEAR(pose[2], 0.05, 1e-9);
  // x, y, z, w: a quarter turn about z.
  EXPECT_NEAR(pose[3], 0.0, 1e-6);
  EXPECT_NEAR(pose[4], 0.0, 1e-6);
  EXPECT_NEAR(pose[5], std::sqrt(0.5), 1e-6);
  EXPECT_NEAR(pose[6], std::sqrt(0.5), 1e-6);
}

TEST(PublishSchedule, PublishesOncePerPeriodOfSimulatedTime)
{
  PublishSchedule schedule(0.02);  // 50 Hz
  int published = 0;
  for (int step = 0; step < 1000; ++step) {  // 1 s at 1 kHz
    published += schedule.due(step * 0.001) ? 1 : 0;
  }
  EXPECT_EQ(published, 50);
}

TEST(PublishSchedule, ASlowStepDoesNotCauseABurst)
{
  PublishSchedule schedule(0.02);
  EXPECT_TRUE(schedule.due(0.0));
  EXPECT_TRUE(schedule.due(0.5));  // the simulation jumped ahead
  EXPECT_FALSE(schedule.due(0.51));
  EXPECT_TRUE(schedule.due(0.52));
}

}  // namespace
