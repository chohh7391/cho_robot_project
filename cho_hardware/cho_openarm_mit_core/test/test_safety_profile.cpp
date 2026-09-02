#include "cho_openarm_mit_core/mit_protocol.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <gtest/gtest.h>

using cho_openarm_mit_core::SafetyBackend;
using cho_openarm_mit_core::load_safety_profile_file;
using cho_openarm_mit_core::load_safety_profile_yaml;

namespace
{
std::string source()
{
  std::ifstream stream(OPENARM_SAFETY_PROFILE_SOURCE);
  std::ostringstream out; out << stream.rdbuf(); return out.str();
}

std::string replace_once(std::string text, const std::string & from, const std::string & to)
{
  const auto at = text.find(from);
  if (at == std::string::npos) throw std::runtime_error("test mutation source not found");
  text.replace(at, from.size(), to); return text;
}
}

TEST(SafetyProfile, ExplicitMujocoSelectionLoadsAndNoDefaultExists)
{
  const auto profile = load_safety_profile_file(
    OPENARM_SAFETY_PROFILE_SOURCE, "mujoco_sim_safe", SafetyBackend::MUJOCO);
  EXPECT_EQ(profile.update_rate_hz, 1000u);
  EXPECT_THROW(load_safety_profile_yaml(source(), "", SafetyBackend::MUJOCO), std::invalid_argument);
}

TEST(SafetyProfile, RealAlwaysRejectsBeforeSocketOpenAndFlagsCannotIndividuallyApprove)
{
  EXPECT_THROW(load_safety_profile_yaml(source(), "real_conservative_unapproved", SafetyBackend::REAL), std::invalid_argument);
  const auto one_flag = replace_once(source(), "hardware_enable_allowed: false\n    update_rate_hz: null", "hardware_enable_allowed: true\n    update_rate_hz: null");
  EXPECT_THROW(load_safety_profile_yaml(one_flag, "real_conservative_unapproved", SafetyBackend::REAL), std::invalid_argument);
  const auto one_status = replace_once(source(), "status: unapproved", "status: prototype_experiment_allowed");
  EXPECT_THROW(load_safety_profile_yaml(one_status, "real_conservative_unapproved", SafetyBackend::REAL), std::invalid_argument);
}

TEST(SafetyProfile, BackendMismatchUnknownAndMissingKeysReject)
{
  EXPECT_THROW(load_safety_profile_yaml(source(), "mujoco_sim_safe", SafetyBackend::REAL), std::invalid_argument);
  EXPECT_THROW(load_safety_profile_yaml(source(), "does_not_exist", SafetyBackend::MUJOCO), std::invalid_argument);
  const auto missing = replace_once(source(), "    status: prototype_experiment_allowed\n", "");
  EXPECT_THROW(load_safety_profile_yaml(missing, "mujoco_sim_safe", SafetyBackend::MUJOCO), std::invalid_argument);
  const auto unknown = replace_once(source(), "    backend: mujoco\n", "    backend: mujoco\n    surprise: 1\n");
  EXPECT_THROW(load_safety_profile_yaml(unknown, "mujoco_sim_safe", SafetyBackend::MUJOCO), std::invalid_argument);
}

TEST(SafetyProfile, NullWrongTypesAndEnumsReject)
{
  const auto null_gain = replace_once(source(), "kp_slew_per_s: [350.0", "kp_slew_per_s: [null");
  EXPECT_THROW(load_safety_profile_yaml(null_gain, "mujoco_sim_safe", SafetyBackend::MUJOCO), std::exception);
  const auto wrong_bool = replace_once(source(), "hardware_enable_allowed: false", "hardware_enable_allowed: 0");
  EXPECT_THROW(load_safety_profile_yaml(wrong_bool, "mujoco_sim_safe", SafetyBackend::MUJOCO), std::invalid_argument);
  const auto wrong_enum = replace_once(source(), "status: prototype_experiment_allowed", "status: approved");
  EXPECT_THROW(load_safety_profile_yaml(wrong_enum, "mujoco_sim_safe", SafetyBackend::MUJOCO), std::invalid_argument);
  const auto defaulted = replace_once(source(), "default_profile: null", "default_profile: mujoco_sim_safe");
  EXPECT_THROW(load_safety_profile_yaml(defaulted, "mujoco_sim_safe", SafetyBackend::MUJOCO), std::invalid_argument);
}
