#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "cho_openarm_mit_core/mit_protocol.hpp"

namespace cho_controller_openarm_mit
{
using namespace cho_openarm_mit_core;

struct DirectArmOwnership
{
  static constexpr std::size_t dof = 7;
  static constexpr std::size_t arm_count = 1;
  static constexpr std::size_t command_count = 39;
  static constexpr std::size_t state_count = 19;

  static std::vector<std::string> command_claims(const std::string & side)
  {
    return complete_claims(side);
  }

  static std::vector<std::string> state_claims(const std::string & side)
  {
    std::vector<std::string> names;
    for (const auto & joint : joint_names(side)) {
      names.push_back(joint + "/position");
      names.push_back(joint + "/velocity");
    }
    const auto arm = "openarm_" + side + "_arm";
    for (const auto * field : {"mit_session_id", "mit_ack_generation",
        "mit_safe_generation", "mit_safe_ack_generation", "mit_status"}) {
      names.push_back(arm + "/" + field);
    }
    return names;
  }
};

struct PairedArmOwnership
{
  static constexpr std::size_t dof = 14;
  static constexpr std::size_t arm_count = 2;
  static constexpr std::size_t command_count = 79;
  static constexpr std::size_t state_count = 39;

  static std::vector<std::string> command_claims()
  {
    auto names = complete_claims("left");
    auto right = complete_claims("right");
    names.insert(names.end(), right.begin(), right.end());
    names.push_back(kPairOwnershipCommand);
    return names;
  }

  static std::vector<std::string> state_claims()
  {
    std::vector<std::string> names;
    for (const auto & side : {std::string("left"), std::string("right")}) {
      for (const auto & joint : joint_names(side)) {
        names.push_back(joint + "/position");
        names.push_back(joint + "/velocity");
      }
    }
    for (const auto & side : {std::string("left"), std::string("right")}) {
      const auto arm = "openarm_" + side + "_arm";
      for (const auto * field : {"mit_session_id", "mit_ack_generation",
          "mit_safe_generation", "mit_safe_ack_generation", "mit_status"}) {
        names.push_back(arm + "/" + field);
      }
    }
    names.push_back(kPairStopReadyState);
    return names;
  }
};
}  // namespace cho_controller_openarm_mit
