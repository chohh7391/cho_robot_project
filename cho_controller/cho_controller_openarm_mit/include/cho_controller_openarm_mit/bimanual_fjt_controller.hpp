#pragma once

#include <memory>
#include <mutex>
#include <atomic>
#include <array>
#include <unordered_map>
#include <optional>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/lock_free_queue.hpp>

#include "cho_openarm_mit_core/mit_protocol.hpp"

namespace cho_controller_openarm_mit
{
using namespace cho_openarm_mit_core;

// MuJoCo's integrated state may settle a few microradians beyond an exact URDF
// boundary. 1e-5 rad (0.000573 deg) admits that numerical residue while being
// far below any meaningful commanded motion or encoder resolution budget.
inline constexpr double kPositionLimitNumericalTolerance = 1e-5;

inline bool within_trajectory_position_limit(
  double value, double lower, double upper)
{
  return value >= lower - kPositionLimitNumericalTolerance &&
         value <= upper + kPositionLimitNumericalTolerance;
}

class BimanualFollowJointTrajectoryController : public controller_interface::ControllerInterface
{
  friend struct BimanualFjtTestAccess;
public:
  explicit BimanualFollowJointTrajectoryController(bool paired = true) : paired_(paired) {}
  using Action = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;
private:
  rclcpp_action::GoalResponse goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal);
  rclcpp_action::CancelResponse cancel(const std::shared_ptr<GoalHandle> goal_handle);
  void accepted(const std::shared_ptr<GoalHandle> goal_handle);
  enum class RunState : std::uint8_t {INACTIVE, SEEDING, READY, EXECUTING, FINAL_HOLD, STOPPING, SAFE_STOPPED, FAULTED};
  enum class StopReason : std::uint8_t {NONE, CANCEL, PREEMPT, CONTROLLED, FAULT};
  enum class TerminalKind : std::uint8_t {SUCCEED, CANCEL, ABORT};
  struct GoalData
  {
    std::uint64_t id{0};
    trajectory_msgs::msg::JointTrajectory trajectory;
    std::array<double, 14> path_position{}, path_velocity{}, goal_position{}, goal_velocity{};
    double goal_time{0.0};
  };
  struct TerminalEvent {std::uint64_t id; TerminalKind kind; int32_t code; std::uint8_t message;};
  struct FeedbackSnapshot
  {
    std::uint64_t id{0};
    std::array<double, 14> desired_position{}, desired_velocity{}, actual_position{}, actual_velocity{};
  };
  bool protocol_ok() const;
  std::size_t dof() const {return paired_ ? 14U : 7U;}
  std::size_t arm_count() const {return paired_ ? 2U : 1U;}
  std::size_t protocol_offset() const {return 2U * dof();}
  std::size_t protocol_index(std::size_t arm, std::size_t field) const
  {return protocol_offset() + arm * 5U + field;}
  std::vector<std::string> canonical_joint_names() const;
  bool canonicalize(const trajectory_msgs::msg::JointTrajectory & input,
    trajectory_msgs::msg::JointTrajectory & output) const;
  void write_pair(
    const std::array<double, 14> & position, const std::array<double, 14> & velocity,
    double stiffness, double damping);
  void request_stop(StopReason reason);
  void finish_stop();
  bool queue_terminal(TerminalKind kind, int32_t code, std::uint8_t message);
  bool queue_terminal_for(std::uint64_t id, TerminalKind kind, int32_t code, std::uint8_t message);
  bool flush_terminal_retry();
  void start_goal(const GoalData * goal, const rclcpp::Time & now);
  void non_realtime_tick();
  bool sample(double elapsed, std::array<double, 14> & q, std::array<double, 14> & dq) const;
  std::array<double, 14> measured_position() const;
  std::mutex handles_mutex_;
  BimanualTrajectoryGate gate_;
  std::unordered_map<std::uint64_t, std::shared_ptr<GoalHandle>> goal_handles_;
  std::unordered_map<std::uint64_t, std::shared_ptr<const GoalData>> goal_data_registry_;
  std::unordered_map<std::uint64_t, std::uint64_t> retirement_cycles_;
  // Three non-RT-owned generations cover active, accepted-pending, and the
  // most recently retired buffer value.  A slot is only reused after two
  // later accepted goals, by which point RT necessarily observed a newer
  // buffer value.  Destruction/reallocation therefore stays off update().
  std::array<std::shared_ptr<const GoalData>, 3> goal_storage_{};
  realtime_tools::RealtimeBuffer<const GoalData *> goal_buffer_{nullptr};
  realtime_tools::LockFreeSPSCQueue<TerminalEvent, 16> terminal_queue_;
  // At most one active and one accepted pending goal exist.  Two durable RT
  // outbox slots therefore cover every simultaneous terminal result even
  // while the RT-to-non-RT SPSC ring is full.
  std::array<std::optional<TerminalEvent>, 2> terminal_retry_{};
  std::optional<TerminalEvent> nonrt_terminal_retry_;
  realtime_tools::LockFreeSPSCQueue<FeedbackSnapshot, 8> feedback_queue_;
  const GoalData * current_goal_{nullptr};
  const GoalData * pending_goal_{nullptr};
  rclcpp_action::Server<Action>::SharedPtr action_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr safe_stop_service_;
  rclcpp::TimerBase::SharedPtr non_realtime_timer_;
  std::atomic<bool> active_{false};
  std::atomic<bool> pending_reserved_{false};
  std::atomic<bool> controlled_stop_requested_{false};
  std::array<std::atomic<std::uint64_t>, 2> cancel_requested_{{0, 0}};
  std::atomic<std::uint64_t> next_goal_id_{1};
  std::atomic<std::size_t> outstanding_terminal_count_{0};
  std::atomic<std::uint64_t> rt_cycle_{0};
  std::atomic<std::uint8_t> public_run_state_{static_cast<std::uint8_t>(RunState::INACTIVE)};
  std::uint64_t last_goal_buffer_id_{0};
  std::uint64_t active_goal_id_{0};
  RunState run_state_{RunState::INACTIVE};
  StopReason stop_reason_{StopReason::NONE};
  rclcpp::Time trajectory_start_{0, 0, RCL_ROS_TIME};
  std::array<double, 14> goal_tolerance_{};
  std::array<double, 14> path_tolerance_{};
  std::array<double, 14> goal_velocity_tolerance_{};
  std::array<double, 14> path_velocity_tolerance_{};
  std::array<double, 14> last_command_{};
  std::array<double, 14> last_velocity_{};
  std::uint64_t generation_{0};
  std::uint64_t session_{0};
  std::uint64_t safe_generation_at_stop_{0};
  std::uint64_t safe_request_generation_{0};
  std::size_t handshake_cycles_{0};
  bool seed_written_{false};
  double stiffness_{10.0};
  double damping_{0.5};
  double lease_cycles_{5.0};
  double default_path_tolerance_{0.2};
  double default_goal_tolerance_{0.02};
  double goal_time_tolerance_{0.5};
  double current_goal_time_tolerance_{0.5};
  int max_handshake_cycles_{100};
  SafetyProfile safety_profile_{};
  // Only meaningful when paired_: the torso's two arms have different joint 1
  // and joint 2 windows, so the right half of a 14-axis trajectory cannot be
  // checked against the left arm's limits.
  SafetyProfile right_safety_profile_{};
  const SafetyProfile & window_for(const std::size_t index) const
  {
    return (paired_ && index >= kJointsPerArm) ? right_safety_profile_ : safety_profile_;
  }
  const bool paired_;
  std::string side_{"left"};
};
}  // namespace cho_controller_openarm_mit
