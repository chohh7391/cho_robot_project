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

#include "cho_hardware_openarm_mit_mujoco/mit_mujoco_system.hpp"

#include <algorithm>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>
namespace cho_hardware_openarm_mit_mujoco
{
namespace
{
// Two commit generations are the same commit when equal, NaN included.
bool same_generation(double a, double b) { return a == b || (std::isnan(a) && std::isnan(b)); }
}  // namespace
hardware_interface::CallbackReturn MitMujocoSystem::on_init(
  const hardware_interface::HardwareInfo & i)
{
  if (MujocoSystemInterface::on_init(i) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;
  auto get = [&](const char * k) {
    auto p = i.hardware_parameters.find(k);
    return p == i.hardware_parameters.end() ? std::string{} : p->second;
  };
  try {
    auto file = get("mit_safety_profile_file"), name = get("mit_safety_profile"),
         rate_text = get("mit_expected_update_rate_hz");
    if (file.empty() || name.empty() || rate_text.empty())
      return hardware_interface::CallbackReturn::ERROR;
    auto rate = std::stoul(rate_text);
    bool bi = std::any_of(i.joints.begin(), i.joints.end(), [](auto & j) {
      return j.name.find("openarm_left_joint") == 0;
    });
    for (auto side :
         bi ? std::vector<std::string>{"left", "right"} : std::vector<std::string>{""}) {
      Arm a;
      a.side = side;
      // One profile per arm, not one for the pair: the torso gives its two arms
      // different joint 1 and joint 2 windows, so a shared limiter would gate
      // one arm against the other's stops.
      a.resource = side.empty() ? "openarm_arm" : "openarm_" + side + "_arm";
      auto p = cho_openarm_mit_core::load_safety_profile_file(
        file, name, cho_openarm_mit_core::SafetyBackend::MUJOCO, side.empty() ? "single" : side);
      if (rate != p.update_rate_hz) return hardware_interface::CallbackReturn::ERROR;
      auto names = cho_openarm_mit_core::joint_names(side);
      for (std::size_t j = 0; j < N; ++j) {
        a.joints[j] = names[j];
        if (std::count_if(i.joints.begin(), i.joints.end(), [&](auto & x) {
              return x.name == names[j];
            }) != 1)
          return hardware_interface::CallbackReturn::ERROR;
      }
      a.limiter = std::make_unique<Limiter>(p, rate);
      a.shadow = std::make_unique<Limiter>(p, rate);
      // A switch abandoned after a successful prepare opens after one second.
      a.gate.set_expiry_cycles(rate);
      arms_.push_back(std::move(a));
    }
  } catch (const std::exception &) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}
std::vector<hardware_interface::CommandInterface> MitMujocoSystem::export_command_interfaces()
{
  base_commands_ = MujocoSystemInterface::export_command_interfaces();
  std::vector<hardware_interface::CommandInterface> o;
  for (auto & h : base_commands_)
    if (h.get_name().find("finger_joint1/position") != std::string::npos) o.push_back(std::move(h));
  static const std::array<std::string, 5> f{
    "position", "velocity", "stiffness", "damping", "effort"};
  for (auto & a : arms_) {
    for (std::size_t j = 0; j < N; ++j) {
      for (std::size_t k = 0; k < 5; ++k) o.emplace_back(a.joints[j], f[k], &a.command[j][k]);
      auto n = a.joints[j] + "/effort";
      auto x = std::find_if(
        base_commands_.begin(), base_commands_.end(), [&](auto & h) { return h.get_name() == n; });
      if (x == base_commands_.end()) return {};
      a.raw_effort[j] = &*x;
    }
    o.emplace_back(a.resource, "mit_session_echo", &a.protocol[5]);
    o.emplace_back(a.resource, "mit_lease_cycles", &a.protocol[6]);
    o.emplace_back(a.resource, "mit_commit_generation", &a.protocol[7]);
    o.emplace_back(a.resource, "mit_safe_request_generation", &a.safe_request);
  }
  if (arms_.size() == 2)
    o.emplace_back("openarm_bimanual", "mit_pair_ownership", &pair_ownership_token_);
  return o;
}
std::vector<hardware_interface::StateInterface> MitMujocoSystem::export_state_interfaces()
{
  base_states_ = MujocoSystemInterface::export_state_interfaces();
  auto o = base_states_;
  static const std::array<std::string, 5> n{
    "mit_session_id", "mit_ack_generation", "mit_safe_generation", "mit_safe_ack_generation",
    "mit_status"};
  for (auto & a : arms_) {
    for (std::size_t j = 0; j < N; ++j) {
      auto pos = a.joints[j] + "/position", vel = a.joints[j] + "/velocity";
      auto p = std::find_if(
             base_states_.begin(), base_states_.end(),
             [&](auto &h) { return h.get_name() == pos; }),
           v = std::find_if(base_states_.begin(), base_states_.end(), [&](auto &h) {
             return h.get_name() == vel;
           });
      if (p == base_states_.end() || v == base_states_.end()) return {};
      a.position[j] = &*p;
      a.velocity[j] = &*v;
    }
    for (std::size_t k = 0; k < 5; ++k) o.emplace_back(a.resource, n[k], &a.protocol[k]);
  }
  if (arms_.size() == 2)
    o.emplace_back("openarm_bimanual", "mit_pair_stop_ready", &pair_stop_ready_);
  return o;
}
hardware_interface::CallbackReturn MitMujocoSystem::on_activate(const rclcpp_lifecycle::State & s)
{
  if (MujocoSystemInterface::on_activate(s) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;
  if (next_session_ > cho_openarm_mit_core::kMaxExactInteger)
    return hardware_interface::CallbackReturn::ERROR;
  const double session = next_session_++;
  for (auto & a : arms_) {
    std::array<double, N> q{};
    for (std::size_t j = 0; j < N; ++j) {
      if (!a.position[j] || !a.velocity[j] || !a.raw_effort[j])
        return hardware_interface::CallbackReturn::ERROR;
      q[j] = a.position[j]->get_value();
      if (!std::isfinite(q[j])) return hardware_interface::CallbackReturn::ERROR;
    }
    a.limiter->reset(q);
    a.protocol.fill(0);
    a.protocol[0] = session;
    a.submitted = 0;
    a.observed = 0;
    a.gate.reset();
    // A new session starts with no producer input. A SAFE request left from
    // the previous session would read as a new one against a SAFE generation
    // back at 0, and the effort commands must read the fresh hold's tau_ff
    // (0): a producer seeds its first commit from them.
    a.safe_request = 0;
    for (auto & joint : a.command) joint.fill(0);
  }
  pair_ownership_token_ = 0;
  pair_stop_ready_ = 0;
  driving_ = true;
  held_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn MitMujocoSystem::on_deactivate(const rclcpp_lifecycle::State & s)
{
  // As the real adapter's orderly stop: the arm is left in a measured SAFE
  // hold that keeps running (hold_without_producers()) instead of being
  // dropped. Zeroing the torque here dropped it for one cycle, after which the
  // INACTIVE write() went on evaluating producer commits.
  for (auto & a : arms_) {
    a.limiter->request_safe();
    a.protocol[4] = 6;  // DISABLED: no producer input is accepted
  }
  driving_ = false;
  return MujocoSystemInterface::on_deactivate(s);
}
hardware_interface::CallbackReturn MitMujocoSystem::on_shutdown(const rclcpp_lifecycle::State & s)
{
  for (auto & a : arms_) {
    a.limiter->request_safe();
    a.protocol[4] = 6;
  }
  driving_ = false;
  return MujocoSystemInterface::on_shutdown(s);
}
hardware_interface::CallbackReturn MitMujocoSystem::on_error(const rclcpp_lifecycle::State &)
{
  // A fault, as on the real adapter, which disables the motors: no torque.
  try {
    for (auto & a : arms_) {
      a.limiter->fault();
      a.protocol[4] = 5;
      for (std::size_t j = 0; j < N; ++j)
        if (a.raw_effort[j]) a.raw_effort[j]->set_value(0.0);
    }
    driving_ = false;
    held_ = false;
    (void)MujocoSystemInterface::write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0));
  } catch (...) {
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}
hardware_interface::return_type MitMujocoSystem::hold_without_producers(
  const rclcpp::Time & t, const rclcpp::Duration & p)
{
  if (!held_) return MujocoSystemInterface::write(t, p);  // never activated: apply nothing
  for (auto & a : arms_) {
    std::array<double, N> q{}, dq{};
    for (std::size_t j = 0; j < N; ++j) {
      q[j] = a.position[j]->get_value();
      dq[j] = a.velocity[j]->get_value();
      if (!std::isfinite(q[j]) || !std::isfinite(dq[j])) a.limiter->fault();
    }
    const auto tau = a.limiter->update(q, dq, p.seconds());
    for (std::size_t j = 0; j < N; ++j) a.raw_effort[j]->set_value(tau[j]);
  }
  const auto r = MujocoSystemInterface::write(t, p);
  if (r != hardware_interface::return_type::OK)
    for (auto & a : arms_) {
      a.limiter->fault();
      a.protocol[4] = 5;
    }
  return r;
}
hardware_interface::CallbackReturn MitMujocoSystem::on_cleanup(const rclcpp_lifecycle::State & s)
{
  for (auto & a : arms_) {
    a.protocol.fill(0);
    a.protocol[4] = 6;
    a.submitted = 0;
    a.observed = 0;
    a.direct_owned = false;
  }
  paired_owned_ = false;
  return MujocoSystemInterface::on_cleanup(s);
}
hardware_interface::return_type MitMujocoSystem::read(
  const rclcpp::Time & t, const rclcpp::Duration & p)
{
  return MujocoSystemInterface::read(t, p);
}
void MitMujocoSystem::rollback_pending()
{
  pending_pair_ = false;
  pending_clear_pair_ = false;
  pending_direct_.fill(false);
}
std::vector<std::string> MitMujocoSystem::filter_base_claims(
  const std::vector<std::string> & claims) const
{
  std::vector<std::string> out;
  out.reserve(claims.size());
  for (const auto & claim : claims) {
    if (claim == cho_openarm_mit_core::kPairOwnershipCommand) continue;
    bool wrapper_claim = false;
    for (const auto & arm : arms_) {
      auto required = cho_openarm_mit_core::complete_claims(arm.side);
      if (std::find(required.begin(), required.end(), claim) != required.end()) {
        wrapper_claim = true;
        break;
      }
    }
    const bool raw_arm_effort =
      wrapper_claim && claim.size() >= 7 && claim.substr(claim.size() - 7) == "/effort";
    if (raw_arm_effort || !wrapper_claim) out.push_back(claim);
  }
  return out;
}
hardware_interface::return_type MitMujocoSystem::prepare_command_mode_switch(
  const std::vector<std::string> & start, const std::vector<std::string> & stop)
{
  rollback_pending();
  auto fail = [this]() {
    rollback_pending();
    return hardware_interface::return_type::ERROR;
  };
  std::size_t full_starts = 0, full_stops = 0;
  const auto token_starts =
    std::count(start.begin(), start.end(), cho_openarm_mit_core::kPairOwnershipCommand);
  const auto token_stops =
    std::count(stop.begin(), stop.end(), cho_openarm_mit_core::kPairOwnershipCommand);
  if (token_starts > 1 || token_stops > 1) return fail();
  for (std::size_t i = 0; i < arms_.size(); ++i) {
    auto & a = arms_[i];
    auto required = cho_openarm_mit_core::complete_claims(a.side);
    auto count = [&](const auto & list) {
      std::size_t n = 0;
      for (auto & x : required) n += std::count(list.begin(), list.end(), x);
      return n;
    };
    auto starts = count(start), stops = count(stop);
    if ((starts != 0 && starts != required.size()) || (stops != 0 && stops != required.size()))
      return fail();
    full_starts += starts == required.size();
    full_stops += stops == required.size();
    pending_direct_[i] = starts == required.size();
  }
  // A stop within one ownership mode follows the shared SwitchGate rule: it is
  // accepted SAFE or not, and write() puts the arm in SAFE itself. An OWNERSHIP
  // change (direct <-> paired) still needs the old owner in an acknowledged
  // SAFE first: the pair transaction needs both arms' generations aligned,
  // which a hardware SAFE cannot provide.
  const auto acknowledged_safe = [](const Arm & a) {
    return a.protocol[4] == 0 && a.protocol[2] != 0 && a.protocol[2] == a.protocol[3];
  };
  const bool both_direct = arms_.size() == 2 && arms_[0].direct_owned && arms_[1].direct_owned;
  if (token_starts == 1) {
    if (arms_.size() != 2 || full_starts != 2 || paired_owned_ || token_stops != 0) return fail();
    if ((arms_[0].direct_owned || arms_[1].direct_owned) && (!both_direct || full_stops != 2))
      return fail();
    if (both_direct && (!acknowledged_safe(arms_[0]) || !acknowledged_safe(arms_[1])))
      return fail();
    if (
      both_direct && (arms_[0].protocol[2] != arms_[1].protocol[2] ||
                      arms_[0].protocol[3] != arms_[1].protocol[3]))
      return fail();
    pending_pair_ = true;
    pending_direct_.fill(false);
  } else if (paired_owned_) {
    if (full_starts != 0) {
      if (full_starts != 2 || full_stops != 2 || token_stops != 1) return fail();
      if (!acknowledged_safe(arms_[0]) || !acknowledged_safe(arms_[1])) return fail();
      pending_clear_pair_ = true;
    } else if (full_stops != 0) {
      if (full_stops != 2 || token_stops != 1) return fail();
      pending_clear_pair_ = true;
    } else if (token_stops != 0)
      return fail();
  } else if (token_stops != 0)
    return fail();
  auto result = MujocoSystemInterface::prepare_command_mode_switch(
    filter_base_claims(start), filter_base_claims(stop));
  if (result != hardware_interface::return_type::OK) {
    rollback_pending();
    return result;
  }
  // Only once the whole switch is accepted: a refused prepare latches nothing.
  for (auto & a : arms_) a.gate.prepare(start, stop, a.side);
  return hardware_interface::return_type::OK;
}
hardware_interface::return_type MitMujocoSystem::perform_command_mode_switch(
  const std::vector<std::string> & start, const std::vector<std::string> & stop)
{
  // No SAFE precondition here any more: the shared SwitchGate rule accepts a
  // stop of an arm that is not SAFE, and write() safes it.
  auto result = MujocoSystemInterface::perform_command_mode_switch(
    filter_base_claims(start), filter_base_claims(stop));
  if (result != hardware_interface::return_type::OK) {
    rollback_pending();
    return result;
  }
  for (std::size_t i = 0; i < arms_.size(); ++i) {
    auto required = cho_openarm_mit_core::complete_claims(arms_[i].side);
    bool stopping = std::all_of(required.begin(), required.end(), [&](auto & x) {
      return std::count(stop.begin(), stop.end(), x) == 1;
    });
    if (stopping) arms_[i].direct_owned = false;
    if (pending_direct_[i]) arms_[i].direct_owned = true;
  }
  if (pending_clear_pair_) paired_owned_ = false;
  if (pending_pair_) {
    paired_owned_ = true;
    for (auto & a : arms_) a.direct_owned = false;
  }
  rollback_pending();
  // Here, not in write(): controller_manager activates the incoming producer
  // right after this, and it continues its generations from the ack.
  std::array<bool, 2> fence{false, false};
  for (std::size_t i = 0; i < arms_.size(); ++i)
    fence[i] = arms_[i].gate.perform(start, stop, arms_[i].side);
  discard_leftover_commits(fence);
  return result;
}
void MitMujocoSystem::discard_leftover_commits(const std::array<bool, 2> & arms)
{
  const auto exact = [](double g) {
    return std::isfinite(g) && g >= 0 && g <= cho_openarm_mit_core::kMaxExactInteger &&
           std::floor(g) == g;
  };
  std::uint64_t newest = 0;
  std::array<std::uint64_t, 2> leftover{0, 0};
  for (std::size_t i = 0; i < arms_.size(); ++i) {
    auto & a = arms_[i];
    if (!arms[i]) continue;
    // Handled, whatever was written: this value is never evaluated again.
    a.observed = a.protocol[7];
    if (!exact(a.protocol[7])) continue;
    leftover[i] = static_cast<std::uint64_t>(a.protocol[7]);
    newest = std::max(newest, leftover[i]);
  }
  for (std::size_t i = 0; i < arms_.size(); ++i) {
    auto & a = arms_[i];
    const std::uint64_t floor = paired_owned_ ? newest : leftover[i];
    if ((arms[i] || paired_owned_) && floor > a.submitted) {
      a.submitted = floor;
      a.protocol[1] = static_cast<double>(floor);
    }
    // The incoming producer seeds from the effort commands: they must read
    // what the hold applies, not the discarded commit's tau_ff.
    if (arms[i]) publish_held_effort(i);
  }
}
void MitMujocoSystem::publish_held_effort(std::size_t i)
{
  auto & a = arms_[i];
  for (std::size_t j = 0; j < N; ++j) a.command[j][4] = a.limiter->held_effort(j);
}
hardware_interface::return_type MitMujocoSystem::write(
  const rclcpp::Time & t, const rclcpp::Duration & p)
{
  if (!driving_) return hold_without_producers(t, p);
  std::array<bool, 2> valid{true, true}, safe_new{false, false};
  std::array<std::uint64_t, 2> generations{}, safe_values{};
  // The controller-switch rule (SwitchGate). `hold`: this cycle evaluates no
  // producer SAFE request or commit for the arm; `hardware_safe`: the arm is put
  // in measured SAFE by the hardware itself, with a SAFE generation of its own.
  std::array<bool, 2> hold{false, false}, hardware_safe{false, false}, discard{false, false};
  for (std::size_t i = 0; i < arms_.size(); ++i) {
    auto & a = arms_[i];
    const auto cycle = a.gate.on_write();
    discard[i] = cycle.discard;
    hardware_safe[i] = cycle.enter_safe && !(a.limiter->safe() && a.protocol[4] == 0);
    hold[i] = hardware_safe[i] || cycle.closed || cycle.discard;
  }
  discard_leftover_commits(discard);
  if (paired_owned_ && (hold[0] || hold[1])) hold[0] = hold[1] = true;
  for (std::size_t i = 0; i < arms_.size(); ++i) {
    auto & a = arms_[i];
    if (hold[i]) continue;
    double s = a.safe_request;
    bool numeric = std::isfinite(s) && s >= 0 && s <= cho_openarm_mit_core::kMaxExactInteger &&
                   std::floor(s) == s;
    // A request at or below the current SAFE generation is no request, as on
    // the real adapter: the hardware advances that generation itself on a
    // controller switch, past whatever the producer last asked for.
    if (numeric && s <= a.protocol[2]) continue;
    if (!numeric || a.protocol[5] != a.protocol[0])
      valid[i] = false;
    else {
      safe_new[i] = true;
      safe_values[i] = static_cast<std::uint64_t>(s);
    }
  }
  if (paired_owned_ && !hold[0]) {
    const bool safe_event = safe_new[0] || safe_new[1];
    const bool left_commit = !same_generation(arms_[0].protocol[7], arms_[0].observed);
    const bool right_commit = !same_generation(arms_[1].protocol[7], arms_[1].observed);
    const bool commit_event = left_commit || right_commit;
    const double common_session = arms_[0].protocol[0];
    const bool token_ok = std::isfinite(pair_ownership_token_) && pair_ownership_token_ >= 0 &&
                          pair_ownership_token_ <= cho_openarm_mit_core::kMaxExactInteger &&
                          std::floor(pair_ownership_token_) == pair_ownership_token_ &&
                          pair_ownership_token_ == common_session &&
                          arms_[1].protocol[0] == common_session;
    const bool safe_ok =
      !safe_event || (safe_new[0] && safe_new[1] && safe_values[0] == safe_values[1]);
    const bool commit_ok =
      !commit_event ||
      (left_commit && right_commit && arms_[0].protocol[7] == arms_[1].protocol[7] &&
       arms_[0].protocol[5] == common_session && arms_[1].protocol[5] == common_session);
    if ((safe_event || commit_event) && (!token_ok || !safe_ok || !commit_ok))
      valid[0] = valid[1] = false;
  }
  for (std::size_t i = 0; i < arms_.size(); ++i) {
    auto & a = arms_[i];
    *a.shadow = *a.limiter;
    generations[i] = a.submitted;
    if (!valid[i]) continue;
    if (hold[i]) {
      if (hardware_safe[i]) {
        a.shadow->request_safe();
        safe_new[i] = true;
        safe_values[i] = static_cast<std::uint64_t>(a.protocol[2]) + 1;
      }
      continue;
    }
    if (safe_new[i]) {
      a.shadow->request_safe();
      continue;
    }
    double g = a.protocol[7], l = a.protocol[6];
    if (same_generation(g, a.observed)) continue;
    // Evaluated once, valid or not. A NaN or fractional value used to be
    // recorded truncated or not at all, so it was re-evaluated -- INVALID and
    // a fresh SAFE latch -- on every cycle it stayed there.
    a.observed = g;
    bool encoding = std::isfinite(g) && std::isfinite(l) && g > 0 && l > 0 &&
                    g <= cho_openarm_mit_core::kMaxExactInteger &&
                    l <= cho_openarm_mit_core::kMaxExactInteger && std::floor(g) == g &&
                    std::floor(l) == l && a.protocol[5] == a.protocol[0];
    std::array<Tuple, N> c{};
    for (std::size_t j = 0; j < N; ++j)
      c[j] = {a.command[j][0], a.command[j][1], a.command[j][2], a.command[j][3], a.command[j][4]};
    if (
      !encoding ||
      !a.shadow->submit(c, static_cast<std::uint64_t>(g), static_cast<std::uint64_t>(l))) {
      valid[i] = false;
      continue;
    }
    generations[i] = static_cast<std::uint64_t>(g);
  }
  if (paired_owned_ && std::any_of(valid.begin(), valid.begin() + arms_.size(), [](bool x) {
        return !x;
      }))
    for (std::size_t i = 0; i < arms_.size(); ++i) valid[i] = false;
  for (std::size_t i = 0; i < arms_.size(); ++i) {
    if (valid[i]) {
      *arms_[i].limiter = *arms_[i].shadow;
      arms_[i].submitted = generations[i];
      if (safe_new[i]) arms_[i].protocol[4] = 2;
    } else {
      arms_[i].protocol[4] = 4;
      arms_[i].limiter->request_safe();
      safe_new[i] = false;
    }
  }
  for (std::size_t i = 0; i < arms_.size(); ++i) {
    auto & a = arms_[i];
    std::array<double, N> q{}, dq{};
    for (std::size_t j = 0; j < N; ++j) {
      q[j] = a.position[j]->get_value();
      dq[j] = a.velocity[j]->get_value();
      if (!std::isfinite(q[j]) || !std::isfinite(dq[j])) {
        a.limiter->fault();
        valid[i] = false;
      }
    }
    auto tau = a.limiter->update(q, dq, p.seconds());
    for (std::size_t j = 0; j < N; ++j) a.raw_effort[j]->set_value(tau[j]);
  }
  auto r = MujocoSystemInterface::write(t, p);
  for (std::size_t i = 0; i < arms_.size(); ++i) {
    auto & a = arms_[i];
    if (r == hardware_interface::return_type::OK) {
      if (valid[i]) {
        a.protocol[1] = a.submitted;
        if (safe_new[i]) a.protocol[2] = a.protocol[3] = safe_values[i];
        a.protocol[4] = a.limiter->safe() ? 0 : 1;
      }
    } else {
      a.limiter->fault();
      a.protocol[4] = 5;
    }
  }
  // While an arm holds, its effort commands read the tau_ff the hold applies,
  // which is what a producer seeds its first commit from -- not a rejected
  // commit's.
  for (std::size_t i = 0; i < arms_.size(); ++i)
    if (arms_[i].limiter->safe()) publish_held_effort(i);
  pair_stop_ready_ = paired_owned_ && arms_.size() == 2 && arms_[0].protocol[4] == 0 &&
                     arms_[1].protocol[4] == 0 && arms_[0].protocol[2] == arms_[0].protocol[3] &&
                     arms_[1].protocol[2] == arms_[1].protocol[3] &&
                     arms_[0].protocol[3] == arms_[1].protocol[3];
  return r;
}
}  // namespace cho_hardware_openarm_mit_mujoco
PLUGINLIB_EXPORT_CLASS(
  cho_hardware_openarm_mit_mujoco::MitMujocoSystem, hardware_interface::SystemInterface)
