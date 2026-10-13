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

// TaskSpaceQP, the QP of TaskSpaceQPController, on the FR3 + hand model, with
// no ROS graph. Against the formulation it replaced (the SE3 task a level-0
// equality, review S20), built here from the same cho_controller_common tasks:
//
//   - near a singularity (sigma_min < 0.006) every joint acceleration stays
//     inside max_joint_acceleration, where the old answer was far outside;
//   - in the regular workspace the answer is the old one, and on a 1 kHz
//     closed-loop FR3 model the EE tracks a goal as well as it did, with the
//     null-space posture on (the MuJoCo config) and off (the real one);
//   - a goal into a near-singular pose keeps every cycle inside the bounds;
//   - an answer that is not finite is not commanded.
//
// The plant is pinocchio's ABA on the model with the fingers locked, the rotor
// armature and joint damping of the MuJoCo model this project tunes against
// (cho_description_franka/xml/fr3/fr3.xml). The torque law is the controller's:
// M_scaled ddq + nle - joint_damping * dq_f, clipped to the FR3 torque limits.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "cho_controller_common/math/util.hpp"
#include "cho_controller_franka/franka_joint_space_qp_controller_parameters.hpp"
#include "cho_controller_franka/franka_operational_space_controller_parameters.hpp"
#include "cho_controller_franka/franka_task_space_qp_controller_parameters.hpp"
#include "cho_controller_franka/task_space_qp_controller.hpp"

namespace
{
using cho_controller::common::robot::RobotWrapper;
using cho_controller::franka::TaskSpaceQP;
using cho_controller::common::formulation::InverseDynamicsFormulationAccForce;
using cho_controller::common::solver::HQP_STATUS_OPTIMAL;
using cho_controller::common::solver::SOLVER_HQP_EIQUADPROG;
using cho_controller::common::solver::SolverHQPBase;
using cho_controller::common::solver::SolverHQPFactory;
using cho_controller::common::tasks::TaskJointPosture;
using cho_controller::common::tasks::TaskSE3Equality;
using cho_controller::common::trajectory::TrajectorySample;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;

constexpr const char * kEE = "fr3_hand_tcp";

Vector7d v7(double a, double b, double c, double d, double e, double f, double g)
{
  Vector7d q;
  q << a, b, c, d, e, f, g;
  return q;
}

// cho_robot_config/config/franka.yaml, poses.task_home.
const Vector7d kTaskHome = v7(0.0, -0.397, 0.0, -2.382, 0.0, 1.985, 0.785);
const Vector7d kTorqueLimits = v7(87, 87, 87, 87, 12, 12, 12);

std::string fr3_urdf()
{
  std::ifstream file(
    ament_index_cpp::get_package_share_directory("cho_description_franka") + "/urdf/fr3/fr3_franka_hand.urdf");
  std::stringstream text;
  text << file.rdbuf();
  return text.str();
}

// The defaults the controller declares (task_space_qp_controller_parameters.yaml).
franka_task_space_qp_controller::Params defaults()
{
  return franka_task_space_qp_controller::Params();
}

Vector7d as7(const std::vector<double> & values) {return Eigen::Map<const Vector7d>(values.data());}

// Gains of the MuJoCo bringup (posture on) or the real one (posture off),
// cho_bringup_franka/config/{mujoco,real}/controllers.yaml.
TaskSpaceQP::Settings settings_of(bool mujoco)
{
  const auto params = defaults();
  TaskSpaceQP::Settings s;
  s.kp_task << (mujoco ? 200.0 : 100.0), (mujoco ? 200.0 : 100.0), (mujoco ? 200.0 : 100.0), 800, 800, 800;
  s.kd_task << 20, 20, 20, 56.57, 56.57, 56.57;
  s.kp_joint = Vector7d::Constant(400.0);
  s.kd_joint = Vector7d::Constant(40.0);
  s.task_weight = params.task_weight;
  s.acceleration_damping_weight = params.acceleration_damping_weight;
  s.use_nullspace_posture = mujoco;
  s.nullspace_posture_weight = params.nullspace_posture_weight;
  s.max_joint_acceleration = as7(params.max_joint_acceleration);
  return s;
}

// The formulation TaskSpaceQPController had before: SE3 a level-0 equality,
// the posture (if on) a level-1 cost at the same weight.
class OldTaskSpaceQP
{
public:
  OldTaskSpaceQP(RobotWrapper & robot, const TaskSpaceQP::Settings & s)
  : se3_("task-se3", robot, kEE), posture_("task-posture", robot), tsid_("tsid", robot),
    reference_(robot.na())
  {
    se3_.Kp(s.kp_task);
    se3_.Kd(s.kd_task);
    Eigen::VectorXd kp = Eigen::VectorXd::Zero(robot.na()), kd = Eigen::VectorXd::Zero(robot.na());
    kp.head(7) = s.kp_joint;
    kd.head(7) = s.kd_joint;
    posture_.Kp(kp);
    posture_.Kd(kd);
    posture_.setReference(reference_);
    tsid_.addMotionTask(se3_, 1.0, 0);
    if (s.use_nullspace_posture) {
      tsid_.addMotionTask(posture_, s.nullspace_posture_weight, 1);
    }
    solver_.reset(SolverHQPFactory::createNewSolver(SOLVER_HQP_EIQUADPROG, "old"));
  }
  void set_se3_reference(TrajectorySample & sample) {se3_.setReference(sample);}
  void set_posture_reference(const Vector7d & q, const Vector7d & v)
  {
    reference_.pos.head(7) = q;
    reference_.vel.head(7) = v;
    posture_.setReference(reference_);
  }
  TaskSpaceQP::Outcome solve(double t, const Eigen::VectorXd & q, const Eigen::VectorXd & v, Vector7d & ddq)
  {
    ddq.setZero();
    const auto & solution = solver_->solve(tsid_.computeProblemData(t, q, v));
    if (solution.status != HQP_STATUS_OPTIMAL) {return TaskSpaceQP::Outcome::kSolverFailed;}
    ddq = tsid_.getAccelerations(solution).head(7);
    return TaskSpaceQP::Outcome::kSolved;
  }

private:
  TaskSE3Equality se3_;
  TaskJointPosture posture_;
  InverseDynamicsFormulationAccForce tsid_;
  std::unique_ptr<SolverHQPBase> solver_;
  TrajectorySample reference_;
};

class TaskSpaceQPBounds : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    xml_ = new std::string(fr3_urdf());
  }
  static void TearDownTestSuite()
  {
    delete xml_;
    xml_ = nullptr;
  }

  void SetUp() override
  {
    ASSERT_FALSE(xml_->empty());
    robot_ = std::make_unique<RobotWrapper>(*xml_, true, false);
    model_ = robot_->model();
    ASSERT_EQ(model_.nq, 9);
    ee_ = model_.getFrameId(kEE);
    ASSERT_LT(ee_, model_.frames.size());
    lower_ = model_.lowerPositionLimit.head(7).array() + 0.05;
    upper_ = model_.upperPositionLimit.head(7).array() - 0.05;
  }

  Eigen::VectorXd full(const Vector7d & q7) const
  {
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model_.nq);
    q.head(7) = q7;
    q(7) = q(8) = 0.04;
    return q;
  }

  // EE pose and the smallest singular value of the arm's 6x7 EE Jacobian.
  double sigma_min(const Vector7d & q7, pinocchio::SE3 * pose = nullptr) const
  {
    pinocchio::Data data(model_);
    const Eigen::VectorXd q = full(q7);
    pinocchio::computeJointJacobians(model_, data, q);
    pinocchio::updateFramePlacements(model_, data);
    pinocchio::Data::Matrix6x J = pinocchio::Data::Matrix6x::Zero(6, model_.nv);
    pinocchio::getFrameJacobian(model_, data, ee_, pinocchio::LOCAL, J);
    if (pose != nullptr) {*pose = data.oMf[ee_];}
    Eigen::JacobiSVD<Eigen::Matrix<double, 6, 7>> svd(J.leftCols(7));
    return svd.singularValues()(5);
  }

  // A reference at `pose * exp(delta)` at rest.
  static TrajectorySample offset_reference(const pinocchio::SE3 & pose, const Vector6d & delta)
  {
    TrajectorySample sample(12, 6);
    cho_controller::common::math::SE3ToVector(pose * pinocchio::exp6(pinocchio::Motion(delta)), sample.pos);
    return sample;
  }

  static double bound_ratio(const Vector7d & ddq, const Vector7d & bound)
  {
    return (ddq.cwiseAbs().array() / bound.array()).maxCoeff();
  }

  struct Run
  {
    double e_max = 0.0;          // worst EE position error against the reference [m]
    double e_end = 0.0;          // EE position error to the goal at the end [m]
    double max_bound_ratio = 0.0;  // max |ddq_i| / max_joint_acceleration_i over the run
    int not_solved = 0;          // cycles whose answer was not commanded
  };

  // A min-jerk Cartesian reference from the pose at q_start to `goal` over T,
  // then 1.5 s of hold, at 1 kHz. `real`: libfranka-style, the plant
  // compensates gravity and nle is Coriolis only; else nle carries gravity.
  template<typename Qp>
  Run closed_loop(Qp & qp, const Vector7d & q_start, const pinocchio::SE3 & goal, double T, bool real) const
  {
    const auto params = defaults();
    const Vector7d inertia_scale = as7(params.inertia_scale);
    const Vector7d joint_damping = as7(params.joint_damping);
    const double alpha = params.velocity_filter_alpha;
    const Vector7d bound = as7(params.max_joint_acceleration);

    pinocchio::Model plant = pinocchio::buildReducedModel(
      model_, {model_.getJointId("fr3_finger_joint1"), model_.getJointId("fr3_finger_joint2")}, full(q_start));
    plant.armature << 0.195, 0.195, 0.195, 0.195, 0.074, 0.074, 0.074;
    pinocchio::Data plant_data(plant);
    const auto plant_ee = plant.getFrameId(kEE);

    pinocchio::Data data(model_);
    pinocchio::SE3 start;
    sigma_min(q_start, &start);
    const Eigen::Vector3d dp = goal.translation() - start.translation();
    const Eigen::Vector3d w = pinocchio::log3(start.rotation().transpose() * goal.rotation());
    qp.set_posture_reference(q_start, Vector7d::Zero());

    Eigen::VectorXd q7 = q_start, v7 = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd q = full(q_start), v = Eigen::VectorXd::Zero(model_.nv);
    Vector7d dq_f = Vector7d::Zero();
    Run run;
    const double dt = 1e-3;
    const int steps = static_cast<int>((T + 1.5) / dt);
    for (int k = 0; k < steps; ++k) {
      const double t = k * dt;
      const double u = std::min(t, T) / T;
      const double s = u * u * u * (10 - 15 * u + 6 * u * u);
      const double sd = t < T ? 30 * u * u * (1 - u) * (1 - u) / T : 0.0;
      const double sdd = t < T ? 60 * u * (1 - u) * (1 - 2 * u) / (T * T) : 0.0;
      const pinocchio::SE3 reference(start.rotation() * pinocchio::exp3(w * s), start.translation() + dp * s);
      TrajectorySample sample(12, 6);
      cho_controller::common::math::SE3ToVector(reference, sample.pos);
      sample.vel << dp * sd, start.rotation() * w * sd;
      sample.acc << dp * sdd, start.rotation() * w * sdd;
      qp.set_se3_reference(sample);

      q.head(7) = q7;
      v.head(7) = v7;
      robot_->computeAllTerms(data, q, v);
      Vector7d ddq;
      if (qp.solve(t, q, v, ddq) != TaskSpaceQP::Outcome::kSolved) {++run.not_solved;}
      run.max_bound_ratio = std::max(run.max_bound_ratio, bound_ratio(ddq, bound));

      Eigen::Matrix<double, 7, 7> M = data.M.topLeftCorner(7, 7);
      M.diagonal().array() *= inertia_scale.array();
      const Vector7d nle = real ?
        Vector7d((robot_->nonLinearEffects(data) - robot_->GeneralizedGravity(data)).head(7)) :
        Vector7d(robot_->nonLinearEffects(data).head(7));
      dq_f = (1 - alpha) * dq_f + alpha * Vector7d(v7);
      Vector7d tau = M * ddq + nle - joint_damping.cwiseProduct(dq_f);
      tau = tau.array().max(-kTorqueLimits.array()).min(kTorqueLimits.array());

      Eigen::VectorXd applied = tau - 0.21 * v7;
      if (real) {applied += pinocchio::computeGeneralizedGravity(plant, plant_data, q7);}
      const Eigen::VectorXd a = pinocchio::aba(plant, plant_data, q7, v7, applied);
      v7 += a * dt;
      q7 = pinocchio::integrate(plant, q7, v7 * dt);
      pinocchio::forwardKinematics(plant, plant_data, q7);
      pinocchio::updateFramePlacements(plant, plant_data);
      run.e_max = std::max(run.e_max, (plant_data.oMf[plant_ee].translation() - reference.translation()).norm());
      if (!q7.allFinite()) {
        run.e_max = std::numeric_limits<double>::infinity();
        break;
      }
    }
    run.e_end = (plant_data.oMf[plant_ee].translation() - goal.translation()).norm();
    return run;
  }

  static pinocchio::SE3 pose(double x, double y, double z, double qx, double qy, double qz, double qw)
  {
    Eigen::Quaterniond rotation(qw, qx, qy, qz);
    rotation.normalize();
    return pinocchio::SE3(rotation.toRotationMatrix(), Eigen::Vector3d(x, y, z));
  }

  static std::string * xml_;
  std::unique_ptr<RobotWrapper> robot_;
  pinocchio::Model model_;
  pinocchio::FrameIndex ee_{0};
  Vector7d lower_, upper_;
};

std::string * TaskSpaceQPBounds::xml_ = nullptr;

// C20: the practical terms are parameters now; their defaults are the
// literals the three controllers used.
TEST(PracticalTermDefaults, AreTheValuesTheControllersHadHardCoded)
{
  const Vector7d inertia = v7(1, 1, 1, 1, 6, 6, 10);
  const double d = 2.0 * std::sqrt(5.0);
  const Vector7d damping = v7(d, d, d, d, 0.2, 0.2, 0.2);
  const auto check = [&](const auto & params, const char * name) {
      EXPECT_DOUBLE_EQ(params.velocity_filter_alpha, 0.99) << name;
      ASSERT_EQ(params.inertia_scale.size(), 7u) << name;
      ASSERT_EQ(params.joint_damping.size(), 7u) << name;
      EXPECT_TRUE(as7(params.inertia_scale) == inertia) << name;
      EXPECT_LT((as7(params.joint_damping) - damping).cwiseAbs().maxCoeff(), 1e-12) << name;
    };
  check(franka_task_space_qp_controller::Params(), "task_space_qp_controller");
  check(franka_joint_space_qp_controller::Params(), "joint_space_qp_controller");
  check(franka_operational_space_controller::Params(), "operational_space_controller");

  const auto tsqp = franka_task_space_qp_controller::Params();
  EXPECT_TRUE(as7(tsqp.max_joint_acceleration) == v7(15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0));
  EXPECT_DOUBLE_EQ(tsqp.task_weight, 1.0);
  EXPECT_DOUBLE_EQ(tsqp.nullspace_posture_weight, 1e-3);
  EXPECT_FALSE(tsqp.use_nullspace_posture);
  const auto osc = franka_operational_space_controller::Params();
  EXPECT_DOUBLE_EQ(osc.kp_null, 10.0);
  EXPECT_DOUBLE_EQ(osc.kd_null, 1.0);
}

TEST_F(TaskSpaceQPBounds, RefusesSettingsItCannotUse)
{
  auto s = settings_of(false);
  s.max_joint_acceleration(3) = 0.0;
  EXPECT_THROW(TaskSpaceQP(*robot_, kEE, Eigen::Vector3d::Zero(), s, 1e-3), std::invalid_argument);
  s = settings_of(false);
  s.acceleration_damping_weight = 0.0;
  EXPECT_THROW(TaskSpaceQP(*robot_, kEE, Eigen::Vector3d::Zero(), s, 1e-3), std::invalid_argument);
  s = settings_of(false);
  EXPECT_THROW(TaskSpaceQP(*robot_, "no_such_frame", Eigen::Vector3d::Zero(), s, 1e-3), std::invalid_argument);
}

// AC2: near a singularity, a large SE3 demand. The old answer, J^+ b, scales
// with 1 / sigma_min; the new one stays inside the bounds.
TEST_F(TaskSpaceQPBounds, NearASingularityEveryJointAccelerationStaysInsideItsBound)
{
  const auto s = settings_of(false);
  TaskSpaceQP qp(*robot_, kEE, Eigen::Vector3d::Zero(), s, 1e-3);
  qp.use_action_tasks();
  OldTaskSpaceQP old(*robot_, s);

  std::mt19937 rng(7);
  pinocchio::Data data(model_);
  int found = 0;
  double old_worst = 0.0, new_worst = 0.0;
  for (int draw = 0; draw < 400000 && found < 40; ++draw) {
    Vector7d q7;
    for (int j = 0; j < 7; ++j) {q7(j) = std::uniform_real_distribution<double>(lower_(j), upper_(j))(rng);}
    pinocchio::SE3 at;
    if (sigma_min(q7, &at) >= 0.006) {continue;}
    ++found;
    const Eigen::VectorXd q = full(q7);
    const Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);
    // 2 cm and 0.05 rad off on every axis: b = kp * e of 2-40 m/s^2, rad/s^2.
    for (int axis = 0; axis < 6; ++axis) {
      Vector6d delta = Vector6d::Zero();
      delta(axis) = axis < 3 ? 0.02 : 0.05;
      TrajectorySample reference = offset_reference(at, delta);
      qp.set_se3_reference(reference);
      old.set_se3_reference(reference);
      Vector7d ddq, ddq_old;
      ASSERT_EQ(qp.solve(0.0, q, v, ddq), TaskSpaceQP::Outcome::kSolved) << q7.transpose();
      old.solve(0.0, q, v, ddq_old);
      const double ratio = bound_ratio(ddq, s.max_joint_acceleration);
      EXPECT_LE(ratio, 1.0 + 1e-6) << "q = " << q7.transpose() << ", axis " << axis << ", ddq = " << ddq.transpose();
      new_worst = std::max(new_worst, ratio);
      old_worst = std::max(old_worst, bound_ratio(ddq_old, s.max_joint_acceleration));
    }
  }
  ASSERT_GE(found, 20) << "too few configurations with sigma_min < 0.006";
  // The configurations are singular enough to matter: the old answer left the
  // bounds by far.
  EXPECT_GT(old_worst, 10.0);
  std::printf("sigma_min < 0.006, %d configurations: worst max|ddq_i| / bound_i %.3g (was %.3g)\n",
    found, new_worst, old_worst);
}

// In the regular workspace (sigma_min >= 0.15), a demand inside the bounds: the
// damped solve is the old minimum-norm answer to within 1 % (the damping moves
// a direction of singular value sigma by lambda / sigma^2, 0.44 % at 0.15).
TEST_F(TaskSpaceQPBounds, InTheRegularWorkspaceTheAnswerIsTheOldOne)
{
  const auto s = settings_of(false);
  TaskSpaceQP qp(*robot_, kEE, Eigen::Vector3d::Zero(), s, 1e-3);
  qp.use_action_tasks();
  OldTaskSpaceQP old(*robot_, s);

  std::mt19937 rng(5);
  int found = 0;
  double worst = 0.0;
  for (int draw = 0; draw < 100000 && found < 200; ++draw) {
    Vector7d q7;
    for (int j = 0; j < 7; ++j) {q7(j) = std::uniform_real_distribution<double>(lower_(j), upper_(j))(rng);}
    pinocchio::SE3 at;
    if (sigma_min(q7, &at) < 0.15) {continue;}
    const Eigen::VectorXd q = full(q7);
    const Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);
    for (int axis = 0; axis < 6; ++axis) {
      Vector6d delta = Vector6d::Zero();
      delta(axis) = axis < 3 ? 0.002 : 0.001;  // 0.2 m/s^2, 0.8 rad/s^2
      TrajectorySample reference = offset_reference(at, delta);
      qp.set_se3_reference(reference);
      old.set_se3_reference(reference);
      Vector7d ddq, ddq_old;
      ASSERT_EQ(qp.solve(0.0, q, v, ddq), TaskSpaceQP::Outcome::kSolved);
      ASSERT_EQ(old.solve(0.0, q, v, ddq_old), TaskSpaceQP::Outcome::kSolved);
      if (bound_ratio(ddq_old, s.max_joint_acceleration) > 0.9) {continue;}
      ++found;
      const double difference = (ddq - ddq_old).norm() / ddq_old.norm();
      worst = std::max(worst, difference);
      EXPECT_LT(difference, 0.01) << "q = " << q7.transpose() << ", axis " << axis;
    }
  }
  ASSERT_GE(found, 200);
  std::printf("regular workspace: worst |ddq - ddq_old| / |ddq_old| %.3g\n", worst);
}

// "Tracks as well as before", measured: goals in the regular workspace on the
// closed-loop model, worst and final EE error against the old formulation.
// With the null-space posture on, this is also what confines the posture to
// the null space: a plain level-1 posture cost left 24 mm at the end of the
// 0.4 m elbow move.
TEST_F(TaskSpaceQPBounds, OnARegularGoalTheEeTracksAsWellAsBefore)
{
  struct Goal
  {
    const char * name;
    pinocchio::SE3 pose;
    double duration;
  };
  pinocchio::SE3 moderate, wrist;
  sigma_min(v7(0.0, 0.3, 0.0, -1.6, 0.0, 1.9, 0.785), &moderate);
  sigma_min(v7(0.0, 0.0, 0.0, -1.6, 0.0, 3.1, 0.785), &wrist);
  const std::vector<Goal> goals = {
    // The MuJoCo S20 cases: 5 cm up from task_home, and the elbow nearly straight.
    {"task_home +5 cm z", pose(0.42663, 0.0, 0.38190, 1.0, 0.0002, 0.0, 0.0), 3.0},
    {"elbow q4=-0.4", pose(0.44448, 0.0, 0.71195, 1.0, 0.0002, 0.0, 0.0), 4.0},
    {"moderate q4=-1.6, 1.5 s", moderate, 1.5},
    {"wrist q6=3.1, 1.5 s", wrist, 1.5},
  };
  for (const bool mujoco : {true, false}) {
    const auto s = settings_of(mujoco);
    for (const auto & goal : goals) {
      TaskSpaceQP qp(*robot_, kEE, Eigen::Vector3d::Zero(), s, 1e-3);
      qp.use_action_tasks();
      OldTaskSpaceQP old(*robot_, s);
      const Run now = closed_loop(qp, kTaskHome, goal.pose, goal.duration, !mujoco);
      const Run before = closed_loop(old, kTaskHome, goal.pose, goal.duration, !mujoco);
      const std::string label = std::string(goal.name) + (mujoco ? " (mujoco config)" : " (real config)");
      EXPECT_EQ(now.not_solved, 0) << label;
      EXPECT_LE(now.max_bound_ratio, 1.0 + 1e-6) << label;
      // Within 10 % (plus 0.5 mm) of the old worst error, and as close at the end.
      EXPECT_LE(now.e_max, 1.1 * before.e_max + 5e-4) << label << ": " << now.e_max << " vs " << before.e_max;
      EXPECT_LE(now.e_end, before.e_end + 1e-3) << label << ": " << now.e_end << " vs " << before.e_end;
      std::printf("%-40s e_max %6.2f mm (was %6.2f), e_end %6.2f mm (was %6.2f)\n", label.c_str(),
        1e3 * now.e_max, 1e3 * before.e_max, 1e3 * now.e_end, 1e3 * before.e_end);
    }
  }
}

// From task_home into the MuJoCo S20 cases' near-singular goals (sigma_min at
// the goal 0.0024 and 0.0035), 8 s each: every cycle inside the bounds and
// commanded. In this same loop the old formulation asked for 1.4e3 to 3.5e5
// times the bound (not rerun here: it doubles the test's time).
TEST_F(TaskSpaceQPBounds, AGoalIntoANearSingularPoseStaysInsideTheBounds)
{
  const std::vector<pinocchio::SE3> goals = {
    pose(0.32684, -0.35150, 1.16331, 0.14687, -0.21668, 0.26059, 0.92929),
    pose(-0.59210, 0.25719, 0.91399, 0.18945, 0.22596, -0.73915, 0.60557),
  };
  for (const bool mujoco : {true, false}) {
    const auto s = settings_of(mujoco);
    for (std::size_t i = 0; i < goals.size(); ++i) {
      TaskSpaceQP qp(*robot_, kEE, Eigen::Vector3d::Zero(), s, 1e-3);
      qp.use_action_tasks();
      const Run now = closed_loop(qp, kTaskHome, goals[i], 8.0, !mujoco);
      const std::string label = "goal " + std::to_string(i) + (mujoco ? " (mujoco config)" : " (real config)");
      EXPECT_EQ(now.not_solved, 0) << label;
      EXPECT_LE(now.max_bound_ratio, 1.0 + 1e-6) << label;
      EXPECT_TRUE(std::isfinite(now.e_max)) << label;
      std::printf("%-26s max|ddq_i| / bound_i %.3g, e_max %6.1f mm, e_end %6.1f mm\n", label.c_str(),
        now.max_bound_ratio, 1e3 * now.e_max, 1e3 * now.e_end);
    }
  }
}

// HOLD: the posture alone at level 1. Inside the bounds the answer is the
// posture demand; outside it is the demand clipped to them, where the old
// level-0 posture commanded it whole.
TEST_F(TaskSpaceQPBounds, TheHoldClipsThePostureDemandToTheBounds)
{
  const auto s = settings_of(false);
  TaskSpaceQP qp(*robot_, kEE, Eigen::Vector3d::Zero(), s, 1e-3);
  qp.use_hold_tasks();
  const Eigen::VectorXd q = full(kTaskHome);
  const Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);
  pinocchio::Data data(model_);
  robot_->computeAllTerms(data, q, v);

  // 0.01 rad off on joint 1: 400 * 0.01 = 4 rad/s^2, inside 15.
  Vector7d target = kTaskHome;
  target(0) += 0.01;
  qp.set_posture_reference(target, Vector7d::Zero());
  Vector7d ddq;
  ASSERT_EQ(qp.solve(0.0, q, v, ddq), TaskSpaceQP::Outcome::kSolved);
  EXPECT_NEAR(ddq(0), 4.0, 1e-6);
  EXPECT_LT(ddq.tail(6).cwiseAbs().maxCoeff(), 1e-6);

  // 0.1 rad off on joints 2 and 6: 40 rad/s^2 asked, 7.5 and 20 allowed.
  target = kTaskHome;
  target(1) -= 0.1;
  target(5) += 0.1;
  qp.set_posture_reference(target, Vector7d::Zero());
  ASSERT_EQ(qp.solve(0.0, q, v, ddq), TaskSpaceQP::Outcome::kSolved);
  EXPECT_NEAR(ddq(1), -7.5, 1e-6);
  EXPECT_NEAR(ddq(5), 20.0, 1e-6);
}

// A state the QP cannot use is never commanded: the answer is zero, and the
// controller's torque is then nle - damping, the damped hold.
TEST_F(TaskSpaceQPBounds, AnAnswerThatIsNotFiniteIsNotCommanded)
{
  const auto s = settings_of(true);
  TaskSpaceQP qp(*robot_, kEE, Eigen::Vector3d::Zero(), s, 1e-3);
  qp.use_action_tasks();
  pinocchio::SE3 at;
  sigma_min(kTaskHome, &at);
  TrajectorySample reference = offset_reference(at, Vector6d::Zero());
  reference.acc(2) = std::numeric_limits<double>::quiet_NaN();
  qp.set_se3_reference(reference);
  qp.set_posture_reference(kTaskHome, Vector7d::Zero());
  Vector7d ddq = Vector7d::Constant(1.0);
  const auto outcome = qp.solve(0.0, full(kTaskHome), Eigen::VectorXd::Zero(model_.nv), ddq);
  EXPECT_NE(outcome, TaskSpaceQP::Outcome::kSolved) << TaskSpaceQP::describe(outcome);
  EXPECT_TRUE(ddq.isZero(0.0));

  // And the next, sane cycle is solved again.
  reference.acc.setZero();
  qp.set_se3_reference(reference);
  EXPECT_EQ(qp.solve(0.0, full(kTaskHome), Eigen::VectorXd::Zero(model_.nv), ddq), TaskSpaceQP::Outcome::kSolved);
}

}  // namespace
