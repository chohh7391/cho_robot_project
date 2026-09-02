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
  stop_left_pending_ = stop_right_pending_ = false;
  sync_protocol();
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
  stop_left_pending_ = stop_right_pending_ = false;
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
  bool ok = true;
  if (bimanual_) {
    if (!pair_)
      return hardware_interface::return_type::ERROR;
    if (direct_ownership_active_) {
      auto write_arm = [&](ArmConsumer & consumer, auto & protocol, std::size_t offset) {
        const auto command = make(offset, protocol);
        if (protocol[8] > static_cast<double>(consumer.safe_generation())) {
          const bool valid = is_exact_nonnegative_integer(protocol[8]) &&
            protocol[8] == static_cast<double>(consumer.safe_generation() + 1);
          if (!valid) return false;
          consumer.request_safe_transition(true);
          return consumer.submit_safe_transition(true);
        }
        const auto status = consumer.status();
        if (status == MitStatus::SAFE && command.generation == consumer.ack_generation()) return true;
        if (status == MitStatus::ACTIVE && command.generation == consumer.ack_generation())
          return consumer.successful_write_cycle();
        if (status == MitStatus::SAFE || status == MitStatus::ACTIVE)
          return consumer.accept_and_write(
            command, command.generation != static_cast<double>(fail_transport_generation_));
        return status == MitStatus::SAFE_TRANSITION && consumer.submit_safe_transition(true);
      };
      const bool left_ok = write_arm(left_, left_protocol_, 0);
      const bool right_ok = write_arm(right_, right_protocol_, 7);
      ok = left_ok && right_ok;
    } else {
    if (left_protocol_[8] > static_cast<double>(pair_->left().safe_generation()) ||
        right_protocol_[8] > static_cast<double>(pair_->right().safe_generation())) {
      const bool valid = is_exact_nonnegative_integer(left_protocol_[8]) &&
        left_protocol_[8] == right_protocol_[8] && pair_protocol_[0] == left_protocol_[0] &&
        left_protocol_[8] == static_cast<double>(pair_->left().safe_generation() + 1) &&
        right_protocol_[8] == static_cast<double>(pair_->right().safe_generation() + 1);
      if (valid) {pair_->request_safe_transition(true, true, true);ok = pair_->submit_safe_transition(true, true, true);}
      else ok = false;
    } else if (stop_left_pending_ || stop_right_pending_)
      ok = pair_->submit_safe_transition(stop_left_pending_,
                                         stop_right_pending_);
    else if (pair_protocol_[0] != left_protocol_[0] && left_protocol_[7] != 0.0)
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
    auto s = left_.status();
    if (left_protocol_[8] > static_cast<double>(left_.safe_generation())) {
      const bool valid=is_exact_nonnegative_integer(left_protocol_[8])&&
        left_protocol_[8]==static_cast<double>(left_.safe_generation()+1);
      if(valid){left_.request_safe_transition(true);ok=left_.submit_safe_transition(true);}else ok=false;
    } else if (stop_left_pending_)
      ok = left_.submit_safe_transition(true);
    else if (s == MitStatus::SAFE && c.generation == left_.ack_generation())
      ok = true;
    else if (s == MitStatus::ACTIVE && c.generation == left_.ack_generation())
      ok = left_.successful_write_cycle();
    else if (s == MitStatus::SAFE || s == MitStatus::ACTIVE)
      ok = left_.accept_and_write(
          c, c.generation != static_cast<double>(fail_transport_generation_));
    else
      ok = false;
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
          state_[i][0] = left_.submitted().joints[i].position;
          state_[i][1] = left_.submitted().joints[i].velocity;
          state_[i][2] = left_.submitted().joints[i].effort;
        }
        if (right_.status() == MitStatus::ACTIVE) {
          state_[i + 7][0] = right_.submitted().joints[i].position;
          state_[i + 7][1] = right_.submitted().joints[i].velocity;
          state_[i + 7][2] = right_.submitted().joints[i].effort;
        }
      }
    } else if (bimanual_ && pair_ && pair_->left().status() == MitStatus::ACTIVE &&
        pair_->right().status() == MitStatus::ACTIVE) {
      for (size_t i = 0; i < 7; ++i) {
        state_[i][0] = pair_->left().submitted().joints[i].position;
        state_[i][1] = pair_->left().submitted().joints[i].velocity;
        state_[i][2] = pair_->left().submitted().joints[i].effort;
        state_[i + 7][0] = pair_->right().submitted().joints[i].position;
        state_[i + 7][1] = pair_->right().submitted().joints[i].velocity;
        state_[i + 7][2] = pair_->right().submitted().joints[i].effort;
      }
    } else if (!bimanual_ && left_.status() == MitStatus::ACTIVE) {
      for (size_t i = 0; i < 7; ++i) {
        state_[i][0] = left_.submitted().joints[i].position;
        state_[i][1] = left_.submitted().joints[i].velocity;
        state_[i][2] = left_.submitted().joints[i].effort;
      }
    }
  }
  sync_protocol();
  return ok ? hardware_interface::return_type::OK
            : hardware_interface::return_type::ERROR;
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
bool FakeMitSystem::exact_owned_claim(const std::vector<std::string> &c,
                                      const std::string &s) const {
  auto r = complete_claims(s);
  std::set<std::string> owned(r.begin(), r.end());
  std::vector<std::string> f;
  std::copy_if(c.begin(), c.end(), std::back_inserter(f),
               [&](auto &x) { return owned.count(x); });
  return f.empty() || (f.size() == r.size() &&
                       std::set<std::string>(f.begin(), f.end()) == owned);
}
hardware_interface::return_type FakeMitSystem::prepare_command_mode_switch(
    const std::vector<std::string> &start,
    const std::vector<std::string> &stop) {
  bool valid =
      bimanual_ ? exact_owned_claim(start, "left") &&
                      exact_owned_claim(start, "right") &&
                      exact_owned_claim(stop, "left") &&
                      exact_owned_claim(stop, "right")
                : exact_owned_claim(start, single_arm_side_) && exact_owned_claim(stop, single_arm_side_);
  if (!valid)
    return hardware_interface::return_type::ERROR;
  if (bimanual_ && !start.empty()) {
    const bool wants_pair = std::find(
      start.begin(), start.end(), "openarm_bimanual/mit_pair_ownership") != start.end();
    const bool wants_direct = !wants_pair;
    if (ownership_selected_ && direct_ownership_active_ != wants_direct) {
      // A mode change is permitted only as one atomic CM stop/start after the old
      // owner has completed SAFE.  Never infer a transition from command values.
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
  auto any = [&](auto &s) {
    auto r = complete_claims(s);
    return std::any_of(stop.begin(), stop.end(), [&](auto &x) {
      return std::find(r.begin(), r.end(), x) != r.end();
    });
  };
  std::string l = bimanual_ ? "left" : single_arm_side_;
  const ArmConsumer & left_consumer =
    bimanual_ && !direct_ownership_active_ ? pair_->left() : left_;
  stop_left_pending_ = any(l) && left_consumer.status() != MitStatus::SAFE;
  if (bimanual_) {
    std::string r = "right";
    const ArmConsumer & right_consumer = direct_ownership_active_ ? right_ : pair_->right();
    stop_right_pending_ = any(r) && right_consumer.status() != MitStatus::SAFE;
  }
  // Switching claims is not a safety transport.  The active controller must
  // have completed its explicit measured-SAFE handshake before CM reaches
  // prepare_command_mode_switch().
  if (stop_left_pending_ || stop_right_pending_) {
    stop_left_pending_ = stop_right_pending_ = false;
    return hardware_interface::return_type::ERROR;
  }
  sync_protocol();
  return hardware_interface::return_type::OK;
}
hardware_interface::return_type
FakeMitSystem::perform_command_mode_switch(const std::vector<std::string> &,
                                           const std::vector<std::string> &) {
  const ArmConsumer &l = bimanual_ && pair_ && !direct_ownership_active_ ? pair_->left() : left_;
  bool ld =
      !stop_left_pending_ || l.safe_ack_generation() == l.safe_generation();
  bool rd =
      !stop_right_pending_ || (bimanual_ &&
        (direct_ownership_active_ ? right_ : pair_->right()).safe_ack_generation() ==
        (direct_ownership_active_ ? right_ : pair_->right()).safe_generation());
  if (!ld || !rd)
    return hardware_interface::return_type::ERROR;
  stop_left_pending_ = stop_right_pending_ = false;
  return hardware_interface::return_type::OK;
}
} // namespace cho_hardware_openarm_mit_test
PLUGINLIB_EXPORT_CLASS(cho_hardware_openarm_mit_test::FakeMitSystem,
                       hardware_interface::SystemInterface)
