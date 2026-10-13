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

// TaskJointBounds: the box it builds over ddq, what it refuses, and what the
// HQP makes of it at level 0. The model is a three-joint arm with a prismatic
// "finger", written below, so the test needs no description package.
//
// compute() runs on the control thread; it is checked not to allocate by
// interposing malloc from this executable (as test_trajectory_no_alloc does).
#include <atomic>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "cho_controller_common/formulation/inverse_dynamics_formulation_acc.hpp"
#include "cho_controller_common/robot/robot_wrapper.hpp"
#include "cho_controller_common/solver/solver_HQP_factory.hpp"
#include "cho_controller_common/tasks/task_joint_bounds.hpp"
#include "cho_controller_common/tasks/task_joint_posture.hpp"

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
using cho_controller::common::formulation::InverseDynamicsFormulationAccForce;
using cho_controller::common::robot::RobotWrapper;
using cho_controller::common::solver::HQP_STATUS_OPTIMAL;
using cho_controller::common::solver::SOLVER_HQP_EIQUADPROG;
using cho_controller::common::solver::SolverHQPBase;
using cho_controller::common::solver::SolverHQPFactory;
using cho_controller::common::tasks::TaskJointBounds;
using cho_controller::common::tasks::TaskJointPosture;
using cho_controller::common::trajectory::TrajectorySample;
using Eigen::VectorXd;

constexpr double kInf = std::numeric_limits<double>::infinity();

const char * kUrdf = R"(<?xml version="1.0"?>
<robot name="bounds_test">
  <link name="base"/>
  <link name="l1"><inertial><mass value="1"/><origin xyz="0 0 0.1"/>
    <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/></inertial></link>
  <link name="l2"><inertial><mass value="1"/><origin xyz="0.1 0 0"/>
    <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/></inertial></link>
  <link name="l3"><inertial><mass value="0.5"/><origin xyz="0.1 0 0"/>
    <inertia ixx="0.005" iyy="0.005" izz="0.005" ixy="0" ixz="0" iyz="0"/></inertial></link>
  <link name="finger"><inertial><mass value="0.05"/>
    <inertia ixx="1e-5" iyy="1e-5" izz="1e-5" ixy="0" ixz="0" iyz="0"/></inertial></link>
  <joint name="j1" type="revolute"><parent link="base"/><child link="l1"/>
    <axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="10" velocity="2"/></joint>
  <joint name="j2" type="revolute"><parent link="l1"/><child link="l2"/><origin xyz="0 0 0.2"/>
    <axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="10" velocity="2"/></joint>
  <joint name="j3" type="revolute"><parent link="l2"/><child link="l3"/><origin xyz="0.2 0 0"/>
    <axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="10" velocity="2"/></joint>
  <joint name="f" type="prismatic"><parent link="l3"/><child link="finger"/><origin xyz="0.2 0 0"/>
    <axis xyz="0 1 0"/><limit lower="0" upper="0.04" effort="10" velocity="0.1"/></joint>
</robot>)";

class TaskJointBoundsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    robot_ = std::make_unique<RobotWrapper>(kUrdf, true, false);
    ASSERT_EQ(robot_->nv(), 4);
    ASSERT_EQ(robot_->na(), 4);
    data_ = std::make_unique<pinocchio::Data>(robot_->model());
    q_ = VectorXd::Zero(robot_->nq());
    v_ = VectorXd::Zero(robot_->nv());
    robot_->computeAllTerms(*data_, q_, v_);
  }

  // Arm joints bounded by `a`, the finger left free.
  static void arm_bounds(TaskJointBounds & task, const VectorXd & a)
  {
    VectorXd upper(4);
    upper << a(0), a(1), a(2), kInf;
    task.setAccelerationBounds(-upper, upper);
  }

  std::unique_ptr<RobotWrapper> robot_;
  std::unique_ptr<pinocchio::Data> data_;
  VectorXd q_, v_;
};

TEST_F(TaskJointBoundsTest, IsABoundOverEveryAccelerationAndUnboundedByDefault)
{
  TaskJointBounds task("bounds", *robot_, 1e-3);
  EXPECT_EQ(task.dim(), robot_->nv());
  const auto & c = task.compute(0.0, q_, v_, *data_);
  EXPECT_TRUE(c.isBound());
  EXPECT_EQ(static_cast<int>(c.rows()), robot_->nv());
  EXPECT_TRUE((c.lowerBound().array() == -TaskJointBounds::kUnbounded).all());
  EXPECT_TRUE((c.upperBound().array() == TaskJointBounds::kUnbounded).all());
}

TEST_F(TaskJointBoundsTest, AccelerationBoundsLandOnTheirJointsAndInfinityMeansFree)
{
  TaskJointBounds task("bounds", *robot_, 1e-3);
  VectorXd lower(4), upper(4);
  lower << -1.0, -2.0, -3.0, -kInf;
  upper << 1.5, 2.5, 3.5, kInf;
  task.setAccelerationBounds(lower, upper);
  const auto & c = task.compute(0.0, q_, v_, *data_);
  for (int i = 0; i < 3; ++i) {
    EXPECT_DOUBLE_EQ(c.lowerBound()(i), lower(i)) << i;
    EXPECT_DOUBLE_EQ(c.upperBound()(i), upper(i)) << i;
  }
  // Infinity is stored finite, so the dense solver never sees it.
  EXPECT_DOUBLE_EQ(c.lowerBound()(3), -TaskJointBounds::kUnbounded);
  EXPECT_DOUBLE_EQ(c.upperBound()(3), TaskJointBounds::kUnbounded);
  EXPECT_TRUE(c.lowerBound().allFinite() && c.upperBound().allFinite());
  // Without velocity bounds the joint velocity changes nothing.
  v_ << 100.0, -100.0, 5.0, 1.0;
  const auto & moving = task.compute(0.0, q_, v_, *data_);
  EXPECT_DOUBLE_EQ(moving.upperBound()(0), 1.5);
  EXPECT_DOUBLE_EQ(moving.lowerBound()(1), -2.0);
}

TEST_F(TaskJointBoundsTest, RefusesWhatItCannotUse)
{
  EXPECT_THROW(TaskJointBounds("bounds", *robot_, 0.0), std::invalid_argument);
  EXPECT_THROW(TaskJointBounds("bounds", *robot_, -1e-3), std::invalid_argument);
  EXPECT_THROW(TaskJointBounds("bounds", *robot_, std::nan("")), std::invalid_argument);
  TaskJointBounds task("bounds", *robot_, 1e-3);
  // One entry per actuated joint, the fingers included.
  EXPECT_THROW(task.setAccelerationBounds(-VectorXd::Ones(3), VectorXd::Ones(3)), std::invalid_argument);
  VectorXd lower = -VectorXd::Ones(4), upper = VectorXd::Ones(4);
  lower(1) = std::nan("");
  EXPECT_THROW(task.setAccelerationBounds(lower, upper), std::invalid_argument);
  lower(1) = 2.0;  // above its upper
  EXPECT_THROW(task.setAccelerationBounds(lower, upper), std::invalid_argument);
  EXPECT_THROW(task.setVelocityBounds(VectorXd::Ones(4), -VectorXd::Ones(4)), std::invalid_argument);
  EXPECT_THROW(task.setTimeStep(0.0), std::invalid_argument);
  EXPECT_THROW(task.setMask(VectorXd::Ones(4)), std::logic_error);
  // A refused call leaves the previous bounds in place.
  EXPECT_TRUE((task.getAccelerationUpperBounds().array() == TaskJointBounds::kUnbounded).all());
}

TEST_F(TaskJointBoundsTest, AVelocityBoundTightensTheBoxWithoutEmptyingIt)
{
  const double dt = 0.01;
  TaskJointBounds task("bounds", *robot_, dt);
  arm_bounds(task, VectorXd::Constant(3, 10.0));
  VectorXd v_max(4);
  v_max << 1.0, 1.0, 1.0, kInf;
  task.setVelocityBounds(-v_max, v_max);

  // 0.95 rad/s with 1 rad/s allowed: at most (1 - 0.95) / dt = 5 rad/s^2 more.
  v_ << 0.95, 0.0, -0.95, 0.0;
  const auto & near = task.compute(0.0, q_, v_, *data_);
  EXPECT_NEAR(near.upperBound()(0), 5.0, 1e-9);
  EXPECT_DOUBLE_EQ(near.lowerBound()(0), -10.0);
  EXPECT_DOUBLE_EQ(near.upperBound()(1), 10.0);
  EXPECT_NEAR(near.lowerBound()(2), -5.0, 1e-9);
  EXPECT_DOUBLE_EQ(near.upperBound()(3), TaskJointBounds::kUnbounded);

  // Already 2 rad/s, past the bound by more than a_max * dt: decelerate at
  // a_max, never an empty box (TSID asked for -100 here).
  v_ << 2.0, 0.0, -2.0, 0.0;
  const auto & past = task.compute(0.0, q_, v_, *data_);
  EXPECT_DOUBLE_EQ(past.upperBound()(0), -10.0);
  EXPECT_DOUBLE_EQ(past.lowerBound()(0), -10.0);
  EXPECT_DOUBLE_EQ(past.lowerBound()(2), 10.0);
  EXPECT_DOUBLE_EQ(past.upperBound()(2), 10.0);

  std::mt19937 rng(3);
  std::uniform_real_distribution<double> speed(-5.0, 5.0);
  for (int k = 0; k < 1000; ++k) {
    for (int i = 0; i < 4; ++i) {v_(i) = speed(rng);}
    const auto & c = task.compute(0.0, q_, v_, *data_);
    ASSERT_TRUE((c.lowerBound().array() <= c.upperBound().array()).all()) << "empty box at v = " << v_.transpose();
    ASSERT_TRUE((c.upperBound().head(3).array() <= 10.0).all() && (c.lowerBound().head(3).array() >= -10.0).all());
  }
}

TEST_F(TaskJointBoundsTest, ComputeDoesNotAllocate)
{
  TaskJointBounds task("bounds", *robot_, 1e-3);
  arm_bounds(task, VectorXd::Constant(3, 10.0));
  task.setVelocityBounds(-VectorXd::Ones(4), VectorXd::Ones(4));
  task.compute(0.0, q_, v_, *data_);
  g_allocations = 0;
  g_counting = true;
  for (int k = 0; k < 100; ++k) {
    v_(0) = 0.01 * k;
    task.compute(0.0, q_, v_, *data_);
  }
  g_counting = false;
  EXPECT_EQ(g_allocations.load(), 0);
}

// At level 0 under a posture cost: the answer is the posture's demand while it
// is inside the box, and the demand clipped to the box when it is not.
TEST_F(TaskJointBoundsTest, AtLevelZeroTheHqpAnswerStaysInsideTheBox)
{
  InverseDynamicsFormulationAccForce tsid("tsid", *robot_);
  TaskJointBounds bounds("bounds", *robot_, 1e-3);
  VectorXd a_max(3);
  a_max << 2.0, 3.0, 4.0;
  arm_bounds(bounds, a_max);
  TaskJointPosture posture("posture", *robot_);
  posture.Kp(VectorXd::Constant(4, 100.0));
  posture.Kd(VectorXd::Zero(4));
  tsid.addMotionTask(bounds, 1.0, 0);
  tsid.addMotionTask(posture, 1.0, 1);
  std::unique_ptr<SolverHQPBase> solver(SolverHQPFactory::createNewSolver(SOLVER_HQP_EIQUADPROG, "qp"));

  TrajectorySample reference(4);
  // Demand -100 * (q - ref): 1, -20, 2.5 and (the free finger) -50 rad/s^2.
  reference.pos << 0.01, -0.2, 0.025, -0.5;
  posture.setReference(reference);
  const auto & solution = solver->solve(tsid.computeProblemData(0.0, q_, v_));
  ASSERT_EQ(solution.status, HQP_STATUS_OPTIMAL);
  const VectorXd ddq = tsid.getAccelerations(solution);
  EXPECT_NEAR(ddq(0), 1.0, 1e-6) << "inside the box: the demand itself";
  EXPECT_NEAR(ddq(1), -3.0, 1e-6) << "outside: clipped to the bound";
  EXPECT_NEAR(ddq(2), 2.5, 1e-6);
  EXPECT_NEAR(ddq(3), -50.0, 1e-6) << "the finger is not bounded";
}

}  // namespace
