#include <gtest/gtest.h>
#include "cho_controller_openarm_mit/direct_mit_controller.hpp"
#include "cho_controller_openarm_mit/safety_backend.hpp"
using namespace cho_controller_openarm_mit;
using namespace cho_openarm_mit_core;
namespace {std::array<double,7> fill(double x){std::array<double,7>a{};a.fill(x);return a;}}

TEST(DirectMitMapping, PositionVelocityAndImpedanceContracts)
{
  DirectMitTarget t;t.position=fill(1);t.velocity=fill(2);t.feedforward=fill(3);
  const auto measured=fill(4),kp=fill(5),kd=fill(6),limit=fill(10);
  auto p=map_direct_mit_command(DirectMitMode::POSITION,t,measured,kp,kd,limit);
  EXPECT_EQ(p.joints[0].position,1);EXPECT_EQ(p.joints[0].velocity,0);EXPECT_EQ(p.joints[0].stiffness,5);EXPECT_EQ(p.joints[0].damping,6);
  auto v=map_direct_mit_command(DirectMitMode::VELOCITY,t,measured,kp,kd,limit);
  EXPECT_EQ(v.joints[0].position,4);EXPECT_EQ(v.joints[0].velocity,2);EXPECT_EQ(v.joints[0].stiffness,0);EXPECT_EQ(v.joints[0].damping,6);
  auto i=map_direct_mit_command(DirectMitMode::IMPEDANCE,t,measured,kp,kd,limit);
  EXPECT_EQ(i.joints[0].position,1);EXPECT_EQ(i.joints[0].velocity,2);EXPECT_EQ(i.joints[0].stiffness,5);EXPECT_EQ(i.joints[0].damping,6);EXPECT_EQ(i.joints[0].effort,3);
}

TEST(DirectMitMapping, PureAndDampedTorqueSemantics)
{
  DirectMitTarget t;t.feedforward=fill(3);auto m=fill(4),kp=fill(5),kd=fill(6),limit=fill(10);
  auto pure=map_direct_mit_command(DirectMitMode::DIRECT_TORQUE,t,m,kp,kd,limit);
  EXPECT_EQ(pure.joints[0].position,4);EXPECT_EQ(pure.joints[0].stiffness,0);EXPECT_EQ(pure.joints[0].damping,0);EXPECT_EQ(pure.joints[0].effort,3);
  auto damped=map_direct_mit_command(DirectMitMode::DAMPED_TORQUE,t,m,kp,kd,limit);
  EXPECT_EQ(damped.joints[0].stiffness,0);EXPECT_EQ(damped.joints[0].damping,6);
}

TEST(DirectMitMapping, CompensationIsSummedBeforeFinalLimit)
{
  DirectMitTarget t;t.feedforward=fill(8);t.compensation=fill(7);
  auto out=map_direct_mit_command(DirectMitMode::COMPENSATED_TORQUE,t,fill(0),fill(1),fill(2),fill(10));
  EXPECT_EQ(out.joints[0].effort,10);EXPECT_EQ(out.joints[0].stiffness,0);EXPECT_EQ(out.joints[0].damping,2);
  t.feedforward=fill(-8);t.compensation=fill(-7);
  out=map_direct_mit_command(DirectMitMode::COMPENSATED_TORQUE,t,fill(0),fill(1),fill(2),fill(10));EXPECT_EQ(out.joints[0].effort,-10);
}

TEST(DirectMitClaims, LeftAndRightAreIndependentAndComplete)
{
  const auto lc=complete_claims("left"),rc=complete_claims("right");EXPECT_EQ(lc.size(),39);EXPECT_EQ(rc.size(),39);
  for(const auto& n:lc)EXPECT_EQ(std::find(rc.begin(),rc.end(),n),rc.end());
  const auto single=complete_claims("");EXPECT_EQ(single.size(),39);
  EXPECT_EQ(single.front(),"openarm_joint1/position");
  EXPECT_EQ(single.back(),"openarm_arm/mit_safe_request_generation");
}

TEST(DirectMitSafeStop, RejectsOlderInflightAckAndRequiresExactOwnGeneration)
{
  constexpr double requested = 8.0;
  // An older request may finish and report SAFE while this request is still in flight.
  EXPECT_FALSE(exact_safe_stop_ack(requested, 7.0, 7.0, double(MitStatus::SAFE)));
  EXPECT_FALSE(exact_safe_stop_ack(requested, 8.0, 7.0, double(MitStatus::SAFE)));
  EXPECT_FALSE(exact_safe_stop_ack(requested, 7.0, 8.0, double(MitStatus::SAFE)));
  EXPECT_FALSE(exact_safe_stop_ack(requested, 8.0, 8.0, double(MitStatus::SAFE_TRANSITION)));
  EXPECT_TRUE(exact_safe_stop_ack(requested, 8.0, 8.0, double(MitStatus::SAFE)));
}

TEST(ControllerSafetyBackend, OnlyExactMujocoAndRealValuesAreAccepted)
{
  EXPECT_EQ(safety_backend_from_parameter("mujoco"), SafetyBackend::MUJOCO);
  EXPECT_EQ(safety_backend_from_parameter("real"), SafetyBackend::REAL);
  EXPECT_THROW(safety_backend_from_parameter("MuJoCo"), std::invalid_argument);
  EXPECT_THROW(safety_backend_from_parameter("simulation"), std::invalid_argument);
  EXPECT_THROW(safety_backend_from_parameter("real "), std::invalid_argument);
}

TEST(ControllerSafetyBackend, SelectedRealCommissioningProfileUsesRealBackend)
{
  const auto profile = load_safety_profile_file(
    OPENARM_SAFETY_PROFILE_SOURCE, "real_conservative_commissioning",
    safety_backend_from_parameter("real"));
  EXPECT_EQ(profile.backend, SafetyBackend::REAL);
  EXPECT_EQ(profile.name, "real_conservative_commissioning");
  EXPECT_DOUBLE_EQ(profile.command_velocity.front(), 0.15);
}

TEST(ControllerSafetyBackend, UnapprovedRealProfileRemainsRejected)
{
  // The legacy real profile cannot accidentally bypass the commissioning
  // envelope or reach transport setup.
  EXPECT_THROW(
    load_safety_profile_file(
      OPENARM_SAFETY_PROFILE_SOURCE, "real_conservative_unapproved",
      safety_backend_from_parameter("real")),
    std::invalid_argument);
}
