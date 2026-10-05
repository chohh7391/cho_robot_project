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

// A point-to-point goal splits across two threads: the servers set the goal,
// limits and duration on the executor, then seed the start, plan and sample on
// the control thread (JointSpaceServer / TaskSpaceServer::compute()). The
// control-thread half must not touch the heap -- the first goal ever used to,
// sizing the start vector there.
//
// Counted by interposing malloc and friends from this executable, which glibc
// resolves ahead of its own for every library in the process: that catches
// Eigen's allocations inside libcho_controller_common, which an
// EIGEN_RUNTIME_NO_MALLOC build of this file alone would not.
#include <atomic>
#include <cstddef>

#include <gtest/gtest.h>
#include <pinocchio/spatial/se3.hpp>

#include "cho_controller_common/trajectory/trajectory_euclidian.hpp"
#include "cho_controller_common/trajectory/trajectory_se3.hpp"

extern "C" {
void * __libc_malloc(std::size_t);
void * __libc_calloc(std::size_t, std::size_t);
void * __libc_realloc(void *, std::size_t);
void * __libc_memalign(std::size_t, std::size_t);
}

namespace
{
std::atomic<bool> g_counting{false};
std::atomic<long> g_allocations{0};

void note()
{
  if (g_counting.load(std::memory_order_relaxed)) {
    g_allocations.fetch_add(1, std::memory_order_relaxed);
  }
}
}  // namespace

extern "C" {
void * malloc(std::size_t size) {note(); return __libc_malloc(size);}
void * calloc(std::size_t count, std::size_t size) {note(); return __libc_calloc(count, size);}
void * realloc(void * pointer, std::size_t size) {note(); return __libc_realloc(pointer, size);}
int posix_memalign(void ** out, std::size_t alignment, std::size_t size)
{
  note();
  *out = __libc_memalign(alignment, size);
  return *out != nullptr ? 0 : 12;  // ENOMEM
}
void * aligned_alloc(std::size_t alignment, std::size_t size) {note(); return __libc_memalign(alignment, size);}
}

namespace
{
using cho_controller::common::trajectory::CartesianMotionLimits;
using cho_controller::common::trajectory::JointMotionLimits;
using cho_controller::common::trajectory::TrajectoryEuclidianRuckig;
using cho_controller::common::trajectory::TrajectorySE3Ruckig;

// Allocations made while it is alive.
class Count
{
public:
  Count() {g_allocations = 0; g_counting = true;}
  ~Count() {g_counting = false;}
  long allocations() const {return g_allocations.load();}
};

TEST(TrajectoryNoAlloc, TheCounterSeesAnAllocation) {
  long seen = 0;
  {
    Count count;
    Eigen::VectorXd heap(64);
    heap.setZero();
    seen = count.allocations();
  }
  EXPECT_GT(seen, 0) << "malloc is not interposed, so the tests below prove nothing";
}

TEST(TrajectoryNoAlloc, TheFirstJointGoalPlansWithoutAllocating) {
  TrajectoryEuclidianRuckig trajectory("joint");
  JointMotionLimits limits;
  limits.max_velocity = Eigen::VectorXd::Constant(7, 2.0);
  limits.max_acceleration = Eigen::VectorXd::Constant(7, 10.0);
  trajectory.setLimits(limits);
  const Eigen::VectorXd goal = Eigen::VectorXd::Constant(7, 0.3);
  const Eigen::VectorXd start = Eigen::VectorXd::Zero(7);
  // Executor: handle_accepted().
  trajectory.setDuration(3.0);
  trajectory.setGoalSample(goal);
  // Control thread: the goal's first compute() and sample.
  long allocations = 0;
  bool planned = false;
  {
    Count count;
    trajectory.setStartTime(0.0);
    trajectory.setInitSample(start);
    trajectory.setCurrentTime(0.001);
    planned = trajectory.planSucceeded();
    trajectory.computeNext();
    allocations = count.allocations();
  }
  EXPECT_TRUE(planned);
  EXPECT_EQ(allocations, 0);
}

TEST(TrajectoryNoAlloc, TheFirstTaskGoalPlansWithoutAllocating) {
  TrajectorySE3Ruckig trajectory("task");
  trajectory.setLimits(CartesianMotionLimits{});
  pinocchio::SE3 goal = pinocchio::SE3::Identity();
  goal.translation() << 0.0, 0.0, -0.05;
  trajectory.setDuration(3.0);
  long allocations = 0;
  bool planned = false;
  {
    Count count;
    trajectory.setGoalSample(goal);
    trajectory.setStartTime(0.0);
    trajectory.setInitSample(pinocchio::SE3::Identity());
    trajectory.setCurrentTime(0.001);
    planned = trajectory.planSucceeded();
    trajectory.computeNext();
    allocations = count.allocations();
  }
  EXPECT_TRUE(planned);
  EXPECT_EQ(allocations, 0);
}

}  // namespace
