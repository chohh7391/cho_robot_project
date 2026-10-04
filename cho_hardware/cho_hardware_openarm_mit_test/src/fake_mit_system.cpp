#include "cho_hardware_openarm_mit_test/fake_mit_system.hpp"
#include <algorithm>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>
#include <set>
#include <stdexcept>
namespace cho_hardware_openarm_mit_test {
namespace {
double number(const hardware_interface::HardwareInfo &i, const char *n) {
  auto p = i.hardware_parameters.find(n);
  if (p == i.hardware_parameters.end())
    throw std::invalid_argument(n);
  double v = std::stod(p->second);
  if (!std::isfinite(v))
    throw std::invalid_argument(n);
  return v;
}
} // namespace
hardware_interface::CallbackReturn
FakeMitSystem::on_init(const hardware_interface::HardwareInfo &i) {
  if (SystemInterface::on_init(i) !=
      hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;
  try {
    double lease = number(info_, "max_lease_cycles");
    if (!is_exact_nonnegative_integer(lease) || lease == 0)
      throw std::invalid_argument("lease");
    limits_ = {
        number(info_, "max_abs_position"), number(info_, "max_abs_velocity"),
        number(info_, "max_stiffness"),    number(info_, "max_damping"),
        number(info_, "max_abs_effort"),   static_cast<uint64_t>(lease)};
    safe_hold_damping_ = number(info_, "safe_hold_damping");
    double initial_position = 0.0;
    if (const auto initial = info_.hardware_parameters.find("initial_position");
      initial != info_.hardware_parameters.end()) {
      initial_position = std::stod(initial->second);
      if (!std::isfinite(initial_position) ||
        std::abs(initial_position) > limits_.max_abs_position) {
        throw std::invalid_argument("initial_position");
      }
    }
    for (auto & joint_state : state_) joint_state[0] = initial_position;
    // Test knob, default 0: the mirrored "measured" position is the accepted
    // q_des plus this offset -- a plant that never quite reaches its command,
    // like an arm sagging under gravity. A producer that re-latches q_des to
    // the measurement every cycle then walks away by this much per cycle.
    if (const auto offset = info_.hardware_parameters.find("mirror_position_offset");
      offset != info_.hardware_parameters.end()) {
      mirror_position_offset_ = std::stod(offset->second);
      if (!std::isfinite(mirror_position_offset_)) throw std::invalid_argument("mirror_position_offset");
    }
    left_ = ArmConsumer(limits_, safe_hold_damping_);
    right_ = ArmConsumer(limits_, safe_hold_damping_);
    auto f = info_.hardware_parameters.find("fail_transport_generation");
    if (f != info_.hardware_parameters.end()) {
      double v = std::stod(f->second);
      if (!is_exact_nonnegative_integer(v))
        throw std::invalid_argument("fail");
      fail_transport_generation_ = static_cast<uint64_t>(v);
    }
  } catch (const std::exception &) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  bimanual_ = info_.joints.size() == 14;
  if (!bimanual_ && info_.joints.size() != 7)
    return hardware_interface::CallbackReturn::ERROR;
  if (!bimanual_) {
    const auto side = info_.hardware_parameters.find("arm_side");
    single_arm_side_ = side == info_.hardware_parameters.end() ? "" : side->second;
    if (single_arm_side_ != "" && single_arm_side_ != "left" && single_arm_side_ != "right")
      return hardware_interface::CallbackReturn::ERROR;
  }
  std::vector<std::string> a;
  for (auto &j : info_.joints)
    a.push_back(j.name);
  const auto expected = bimanual_ ? [&]() {auto names=joint_names("left");auto right=joint_names("right");names.insert(names.end(),right.begin(),right.end());return names;}() : joint_names(single_arm_side_);
  return a == expected
             ? hardware_interface::CallbackReturn::SUCCESS
             : hardware_interface::CallbackReturn::ERROR;
}
hardware_interface::CallbackReturn
FakeMitSystem::on_configure(const rclcpp_lifecycle::State &) {
  if (next_session_ == 0 ||
      next_session_ > static_cast<uint64_t>(kMaxExactInteger)) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  std::array<double, 7> l{}, r{};
  for (size_t i = 0; i < 7; ++i) {
    l[i] = state_[i][0];
    if (bimanual_)
      r[i] = state_[i + 7][0];
  }
  if (bimanual_) {
    pair_ = std::make_unique<PairedConsumer>(limits_, next_session_,
                                             safe_hold_damping_);
    if (!pair_->configure(next_session_, l, r))
      return hardware_interface::CallbackReturn::ERROR;
    // The bimanual fake is also used to exercise the production ownership
    // split: two disjoint direct controllers use independent consumers while
    // the MoveIt controller exclusively uses PairedConsumer.
    if (!left_.configure(next_session_, l) || !right_.configure(next_session_, r))
      return hardware_interface::CallbackReturn::ERROR;
  } else if (!left_.configure(next_session_, l))
    return hardware_interface::CallbackReturn::ERROR;
  ++next_session_;
  ownership_selected_ = direct_ownership_active_ = false;
  left_gate_.reset();
  right_gate_.reset();
  // A new session starts with no producer input: its hold has no tau_ff, and
  // the effort commands must say so (publish_held_effort()).
  command_ = {};
  left_protocol_.fill(0);
  right_protocol_.fill(0);
  left_observed_ = right_observed_ = 0.0;
  sync_protocol();
  publish_held_effort();
  return hardware_interface::CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn
FakeMitSystem::on_activate(const rclcpp_lifecycle::State &) {
  driving_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}
void FakeMitSystem::stop_holding() {
  auto hold = [](ArmConsumer &c) {
    if (c.status() == MitStatus::ACTIVE || c.status() == MitStatus::STALE)
      c.request_safe_transition(true);
    if (c.status() == MitStatus::SAFE_TRANSITION) c.submit_safe_transition(true);
  };
  hold(left_);
  if (bimanual_) {
    hold(right_);
    if (pair_) {
      const bool l = pair_->left().status() != MitStatus::SAFE;
      const bool r = pair_->right().status() != MitStatus::SAFE;
      if (l || r) {
        pair_->request_safe_transition(l, r, true);
        pair_->submit_safe_transition(l, r, true);
      }
    }
  }
  driving_ = false;
  sync_protocol();
  publish_held_effort();
}
hardware_interface::CallbackReturn
FakeMitSystem::on_deactivate(const rclcpp_lifecycle::State &) {
  stop_holding();
  return hardware_interface::CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn
FakeMitSystem::on_shutdown(const rclcpp_lifecycle::State &) {
  try { stop_holding(); } catch (...) {}
  return hardware_interface::CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn
FakeMitSystem::on_error(const rclcpp_lifecycle::State &) {
  try { stop_holding(); } catch (...) {}
  return hardware_interface::CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn
FakeMitSystem::on_cleanup(const rclcpp_lifecycle::State &) {
  left_.cleanup();
  right_.cleanup();
  pair_.reset();
  ownership_selected_ = direct_ownership_active_ = false;
  command_ = {};
  left_protocol_.fill(0);
  right_protocol_.fill(0);
  pair_protocol_.fill(0);
  left_gate_.reset();
  right_gate_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}
std::vector<hardware_interface::StateInterface>
FakeMitSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> o;
  for (size_t i = 0; i < info_.joints.size(); ++i)
    for (size_t k = 0; k < 3; ++k)
      o.emplace_back(
          info_.joints[i].name,
          std::array<std::string, 3>{"position", "velocity", "effort"}[k],
          &state_[i][k]);
  auto add = [&](auto n, auto &p) {
    std::array<std::string, 5> x{"mit_session_id", "mit_ack_generation",
                                 "mit_safe_generation",
                                 "mit_safe_ack_generation", "mit_status"};
    for (size_t k = 0; k < 5; ++k)
      o.emplace_back(n, x[k], &p[k]);
  };
  add(bimanual_ ? "openarm_left_arm" :
    (single_arm_side_.empty() ? "openarm_arm" : "openarm_" + single_arm_side_ + "_arm"), left_protocol_);
  if (bimanual_) {
    add("openarm_right_arm", right_protocol_);
    o.emplace_back("openarm_bimanual", "mit_pair_stop_ready",
                   &pair_protocol_[1]);
  }
  return o;
}
std::vector<hardware_interface::CommandInterface>
FakeMitSystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> o;
  std::array<std::string, 5> x{"position", "velocity", "stiffness", "damping",
                               "effort"};
  for (size_t i = 0; i < info_.joints.size(); ++i)
    for (size_t k = 0; k < 5; ++k)
      o.emplace_back(info_.joints[i].name, x[k], &command_[i][k]);
  auto add = [&](auto n, auto &p) {
    o.emplace_back(n, "mit_session_echo", &p[5]);
    o.emplace_back(n, "mit_lease_cycles", &p[6]);
    o.emplace_back(n, "mit_commit_generation", &p[7]);
    o.emplace_back(n, "mit_safe_request_generation", &p[8]);
  };
  add(bimanual_ ? "openarm_left_arm" :
    (single_arm_side_.empty() ? "openarm_arm" : "openarm_" + single_arm_side_ + "_arm"), left_protocol_);
  if (bimanual_) {
    add("openarm_right_arm", right_protocol_);
    o.emplace_back("openarm_bimanual", "mit_pair_ownership",
                   &pair_protocol_[0]);
  }
  return o;
}
hardware_interface::return_type FakeMitSystem::read(const rclcpp::Time &,
                                                    const rclcpp::Duration &) {
  return hardware_interface::return_type::OK;
}
hardware_interface::return_type FakeMitSystem::write(const rclcpp::Time &,
                                                     const rclcpp::Duration &) {
  // Not driving (configured, or deactivated): the hold stands, and no
  // producer input is evaluated.
  if (!driving_) return hardware_interface::return_type::OK;
  auto make = [&](size_t off, auto &p) {
    ArmCommand c;
    for (size_t i = 0; i < 7; ++i)
      c.joints[i] = {command_[off + i][0], command_[off + i][1],
                     command_[off + i][2], command_[off + i][3],
                     command_[off + i][4]};
    c.session_echo = p[5];
    c.lease_cycles = p[6];
    c.generation = p[7];
    return c;
  };
  // The fake tracks its commands perfectly, so its state IS the measured pose
  // a SAFE transition must hold (ArmConsumer::observe).
  {
    std::array<double, 7> l{}, r{};
    for (size_t i = 0; i < 7; ++i) {
      l[i] = state_[i][0];
      if (bimanual_) r[i] = state_[i + 7][0];
    }
    left_.observe(l);
    if (bimanual_) {
      right_.observe(r);
      if (pair_) pair_->observe(l, r);
    }
  }
  bool ok = true;
  if (bimanual_) {
    if (!pair_)
      return hardware_interface::return_type::ERROR;
    if (direct_ownership_active_) {
      auto write_arm = [&](ArmConsumer & consumer, auto & protocol, std::size_t offset,
          SwitchGate & gate) {
        const auto command = make(offset, protocol);
        const auto cycle = gate.on_write();
        if (cycle.discard) {
          consumer.discard_commit(protocol[7]);
          (offset == 0 ? left_observed_ : right_observed_) = protocol[7];
        }
        bool entered = true;
        if (cycle.enter_safe && enter_switch_safe(consumer, entered)) return entered;
        if (cycle.closed || cycle.discard) {
          return consumer.status() == MitStatus::SAFE ||
            (consumer.status() == MitStatus::SAFE_TRANSITION && consumer.submit_safe_transition(true));
        }
        if (protocol[8] > static_cast<double>(consumer.safe_generation())) {
          const bool valid = is_exact_nonnegative_integer(protocol[8]) &&
            protocol[8] == static_cast<double>(consumer.safe_generation() + 1);
          if (!valid) return false;
          consumer.request_safe_transition(true);
          return consumer.submit_safe_transition(true);
        }
        return evaluate_commit(consumer, offset == 0 ? left_observed_ : right_observed_, command);
      };
      const bool left_ok = write_arm(left_, left_protocol_, 0, left_gate_);
      const bool right_ok = write_arm(right_, right_protocol_, 7, right_gate_);
      ok = left_ok && right_ok;
    } else {
    // Both arms of a pair are always switched together.
    const auto left_cycle = left_gate_.on_write();
    const auto right_cycle = right_gate_.on_write();
    if (left_cycle.discard || right_cycle.discard) {
      const double newest = std::max(left_protocol_[7], right_protocol_[7]);
      pair_->discard_commit(newest, newest);
    }
    const bool enter_left = left_cycle.enter_safe && pair_->left().status() != MitStatus::SAFE;
    const bool enter_right = right_cycle.enter_safe && pair_->right().status() != MitStatus::SAFE;
    const bool closed = left_cycle.closed || right_cycle.closed || left_cycle.discard ||
      right_cycle.discard;
    if (enter_left || enter_right) {
      // The hardware owns this SAFE, aligned on both arms like any pair SAFE.
      pair_->request_safe_transition(
        enter_left && pair_->left().status() != MitStatus::SAFE_TRANSITION,
        enter_right && pair_->right().status() != MitStatus::SAFE_TRANSITION, true);
      ok = pair_->submit_safe_transition(enter_left, enter_right, true);
    } else if (closed) {
      ok = pair_->left().status() == MitStatus::SAFE && pair_->right().status() == MitStatus::SAFE;
    } else if (left_protocol_[8] > static_cast<double>(pair_->left().safe_generation()) ||
        right_protocol_[8] > static_cast<double>(pair_->right().safe_generation())) {
      const bool valid = is_exact_nonnegative_integer(left_protocol_[8]) &&
        left_protocol_[8] == right_protocol_[8] && pair_protocol_[0] == left_protocol_[0] &&
        left_protocol_[8] == static_cast<double>(pair_->left().safe_generation() + 1) &&
        right_protocol_[8] == static_cast<double>(pair_->right().safe_generation() + 1);
      if (valid) {pair_->request_safe_transition(true, true, true);ok = pair_->submit_safe_transition(true, true, true);}
      else ok = false;
    } else if (pair_protocol_[0] != left_protocol_[0] && left_protocol_[7] != 0.0)
      ok = false;
    else {
      auto l = make(0, left_protocol_), r = make(7, right_protocol_);
      auto ls = pair_->left().status(), rs = pair_->right().status();
      if (ls == MitStatus::SAFE && rs == MitStatus::SAFE &&
          l.generation == pair_->left().ack_generation() &&
          r.generation == pair_->right().ack_generation())
        ok = true;
      else if (ls == MitStatus::ACTIVE && rs == MitStatus::ACTIVE &&
               l.generation == pair_->left().ack_generation() &&
               r.generation == pair_->right().ack_generation())
        ok = pair_->successful_write_cycle();
      else if ((ls == MitStatus::SAFE || ls == MitStatus::ACTIVE) &&
               (rs == MitStatus::SAFE || rs == MitStatus::ACTIVE))
        ok = pair_->write_pair(
            l, r,
            l.generation != static_cast<double>(fail_transport_generation_));
      else
        ok = false;
    }
    if (!ok) {
      bool sl = pair_->left().status() == MitStatus::SAFE_TRANSITION,
           sr = pair_->right().status() == MitStatus::SAFE_TRANSITION;
      if ((sl || sr) && pair_->submit_safe_transition(sl, sr, true))
        ok = true;
    }
    }
  } else {
    auto c = make(0, left_protocol_);
    const auto cycle = left_gate_.on_write();
    if (cycle.discard) {
      left_.discard_commit(left_protocol_[7]);
      left_observed_ = left_protocol_[7];
    }
    auto s = left_.status();
    if (cycle.enter_safe && enter_switch_safe(left_, ok)) {
      // This cycle put the arm in SAFE for a controller switch.
    } else if (cycle.closed || cycle.discard) {
      ok = s == MitStatus::SAFE;
    } else if (left_protocol_[8] > static_cast<double>(left_.safe_generation())) {
      const bool valid=is_exact_nonnegative_integer(left_protocol_[8])&&
        left_protocol_[8]==static_cast<double>(left_.safe_generation()+1);
      if(valid){left_.request_safe_transition(true);ok=left_.submit_safe_transition(true);}else ok=false;
    } else
      ok = evaluate_commit(left_, left_observed_, c);
    if (!ok && left_.status() == MitStatus::SAFE_TRANSITION &&
        left_.submit_safe_transition(true))
      ok = true;
  }
  // This is a non-driving test double: mirror an accepted tuple into measured
  // state so controller convergence can be integration-tested without modeling
  // or emitting actuator effort.
  if (ok) {
    if (bimanual_ && direct_ownership_active_) {
      for (size_t i = 0; i < 7; ++i) {
        if (left_.status() == MitStatus::ACTIVE) {
          state_[i][0] = left_.submitted().joints[i].position + mirror_position_offset_;
          state_[i][1] = left_.submitted().joints[i].velocity;
          state_[i][2] = left_.submitted().joints[i].effort;
        }
        if (right_.status() == MitStatus::ACTIVE) {
          state_[i + 7][0] = right_.submitted().joints[i].position + mirror_position_offset_;
          state_[i + 7][1] = right_.submitted().joints[i].velocity;
          state_[i + 7][2] = right_.submitted().joints[i].effort;
        }
      }
    } else if (bimanual_ && pair_ && pair_->left().status() == MitStatus::ACTIVE &&
        pair_->right().status() == MitStatus::ACTIVE) {
      for (size_t i = 0; i < 7; ++i) {
        state_[i][0] = pair_->left().submitted().joints[i].position + mirror_position_offset_;
        state_[i][1] = pair_->left().submitted().joints[i].velocity;
        state_[i][2] = pair_->left().submitted().joints[i].effort;
        state_[i + 7][0] = pair_->right().submitted().joints[i].position + mirror_position_offset_;
        state_[i + 7][1] = pair_->right().submitted().joints[i].velocity;
        state_[i + 7][2] = pair_->right().submitted().joints[i].effort;
      }
    } else if (!bimanual_ && left_.status() == MitStatus::ACTIVE) {
      for (size_t i = 0; i < 7; ++i) {
        state_[i][0] = left_.submitted().joints[i].position + mirror_position_offset_;
        state_[i][1] = left_.submitted().joints[i].velocity;
        state_[i][2] = left_.submitted().joints[i].effort;
      }
    }
  }
  sync_protocol();
  publish_held_effort();
  return ok ? hardware_interface::return_type::OK
            : hardware_interface::return_type::ERROR;
}
bool FakeMitSystem::evaluate_commit(ArmConsumer &consumer, double &observed,
                                    const ArmCommand &c) {
  // As the real adapter: a commit generation is evaluated once, accepted or
  // not, and on a copy of the consumer first. A rejected one puts the arm in
  // measured SAFE, recoverably -- a later valid generation is evaluated again.
  // The fake used to latch INVALID for the session and re-evaluate the same
  // rejected generation every cycle, failing write() each time.
  const auto status = consumer.status();
  const bool fresh = !(c.generation == observed ||
                       (std::isnan(c.generation) && std::isnan(observed)));
  if (fresh && (status == MitStatus::SAFE || status == MitStatus::ACTIVE)) {
    observed = c.generation;
    if (c.generation == static_cast<double>(fail_transport_generation_))
      return consumer.accept_and_write(c, false);  // transport FAULT, latched
    ArmConsumer shadow = consumer;
    if (shadow.accept_and_write(c, true)) {
      consumer = shadow;
      return true;
    }
    consumer.request_safe_transition(true);
    return consumer.submit_safe_transition(true);
  }
  if (status == MitStatus::SAFE) return true;
  if (status == MitStatus::ACTIVE) return consumer.successful_write_cycle();
  return status == MitStatus::SAFE_TRANSITION && consumer.submit_safe_transition(true);
}
void FakeMitSystem::publish_held_effort() {
  // As the real adapter: while an arm holds, its effort command interfaces
  // read the tau_ff the hold applies (the last accepted one, 0 in a fresh
  // session), which is what a producer seeds its first commit from.
  auto publish = [&](const ArmConsumer &c, std::size_t offset) {
    if (c.status() == MitStatus::ACTIVE) return;
    for (std::size_t i = 0; i < 7; ++i)
      command_[offset + i][4] = c.submitted().joints[i].effort;
  };
  if (bimanual_ && pair_) {
    publish(direct_ownership_active_ ? left_ : pair_->left(), 0);
    publish(direct_ownership_active_ ? right_ : pair_->right(), 7);
  } else
    publish(left_, 0);
}
void FakeMitSystem::sync_protocol() {
  auto s = [](const ArmConsumer &c, auto &p) {
    p[0] = c.session();
    p[1] = c.ack_generation();
    p[2] = c.safe_generation();
    p[3] = c.safe_ack_generation();
    p[4] = static_cast<double>(c.status());
  };
  if (bimanual_ && pair_) {
    const auto & left = direct_ownership_active_ ? left_ : pair_->left();
    const auto & right = direct_ownership_active_ ? right_ : pair_->right();
    s(left, left_protocol_);
    s(right, right_protocol_);
    pair_protocol_[1] = left.status() == MitStatus::SAFE &&
                                right.status() == MitStatus::SAFE &&
                                left.safe_ack_generation() > 0 &&
                                left.safe_ack_generation() == right.safe_ack_generation()
                            ? 1.0
                            : 0.0;
  } else
    s(left_, left_protocol_);
}
bool FakeMitSystem::enter_switch_safe(ArmConsumer & consumer, bool & ok) {
  const auto status = consumer.status();
  if (status == MitStatus::ACTIVE || status == MitStatus::STALE) {
    consumer.request_safe_transition(true);
  } else if (status != MitStatus::SAFE_TRANSITION) {
    return false;  // already SAFE (or latched): nothing to enter
  }
  ok = consumer.submit_safe_transition(true);
  return true;
}
hardware_interface::return_type FakeMitSystem::prepare_command_mode_switch(
    const std::vector<std::string> &start,
    const std::vector<std::string> &stop) {
  const std::string l = bimanual_ ? "left" : single_arm_side_;
  for (const auto &side : bimanual_ ? std::vector<std::string>{l, "right"}
                                    : std::vector<std::string>{l}) {
    if (classify_arm_claim(start, side) == ArmClaim::PARTIAL ||
        classify_arm_claim(stop, side) == ArmClaim::PARTIAL)
      return hardware_interface::return_type::ERROR;
  }
  if (bimanual_ && !start.empty()) {
    const bool wants_pair = std::find(
      start.begin(), start.end(), "openarm_bimanual/mit_pair_ownership") != start.end();
    const bool wants_direct = !wants_pair;
    if (ownership_selected_ && direct_ownership_active_ != wants_direct) {
      // An OWNERSHIP change (direct <-> paired) is still permitted only as one
      // atomic CM stop/start after the old owner has completed SAFE: the pair
      // transaction needs both arms' generations aligned, which a hardware
      // SAFE cannot provide. A stop within one ownership mode follows the
      // shared SwitchGate rule instead.
      const bool old_safe = direct_ownership_active_ ?
        left_.status() == MitStatus::SAFE && right_.status() == MitStatus::SAFE :
        pair_ && pair_->left().status() == MitStatus::SAFE && pair_->right().status() == MitStatus::SAFE;
      if (stop.empty() || !old_safe) return hardware_interface::return_type::ERROR;
    }
    // Select ownership from ControllerManager's atomic claim set. Command
    // values are deliberately irrelevant because commit values are transient.
    direct_ownership_active_ = wants_direct;
    ownership_selected_ = true;
  }
  // Contract v1 "External switch", the rule all three backends share: the
  // outgoing producer is not relied on for safety. A stop of an arm that is not
  // SAFE is accepted, and write() puts the arm in SAFE itself.
  left_gate_.prepare(start, stop, l);
  if (bimanual_) right_gate_.prepare(start, stop, "right");
  sync_protocol();
  return hardware_interface::return_type::OK;
}
hardware_interface::return_type
FakeMitSystem::perform_command_mode_switch(const std::vector<std::string> &start,
                                           const std::vector<std::string> &stop) {
  const std::string l = bimanual_ ? "left" : single_arm_side_;
  const bool fence_left = left_gate_.perform(start, stop, l);
  const bool fence_right = bimanual_ && right_gate_.perform(start, stop, "right");
  if (fence_left || fence_right) {
    // Every consumer the incoming producer could read its ack from: the
    // ownership mode may have just changed.
    if (fence_left) {
      left_.discard_commit(left_protocol_[7]);
      left_observed_ = left_protocol_[7];
    }
    if (fence_right) {
      right_.discard_commit(right_protocol_[7]);
      right_observed_ = right_protocol_[7];
    }
    if (pair_) {
      // The pair's two acks must stay equal (the paired producer requires it),
      // so both move to the newer of the two leftovers.
      double newest = 0.0;
      for (const double g : {left_protocol_[7], right_protocol_[7]})
        if (is_exact_nonnegative_integer(g)) newest = std::max(newest, g);
      pair_->discard_commit(newest, newest);
    }
    sync_protocol();
    // The leftover's tau_ff is not what the hold applies; the incoming
    // producer, activated right after this, must not seed from it.
    auto restore = [&](const ArmConsumer &c, std::size_t offset) {
      for (std::size_t i = 0; i < 7; ++i)
        command_[offset + i][4] = c.submitted().joints[i].effort;
    };
    if (bimanual_ && pair_) {
      restore(direct_ownership_active_ ? left_ : pair_->left(), 0);
      restore(direct_ownership_active_ ? right_ : pair_->right(), 7);
    } else
      restore(left_, 0);
  }
  return hardware_interface::return_type::OK;
}
} // namespace cho_hardware_openarm_mit_test
PLUGINLIB_EXPORT_CLASS(cho_hardware_openarm_mit_test::FakeMitSystem,
                       hardware_interface::SystemInterface)
