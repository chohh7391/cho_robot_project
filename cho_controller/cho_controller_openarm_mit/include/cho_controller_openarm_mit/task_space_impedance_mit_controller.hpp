#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <unordered_map>

#include <cho_interfaces/action/task_space.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <realtime_tools/lock_free_queue.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include "cho_controller_openarm_mit/direct_mit_controller.hpp"

namespace cho_controller_openarm_mit
{
// Cartesian single-arm MIT impedance action controller.  It deliberately
// remains a direct-controller vertical: MoveIt/FJT owns only paired 14-axis
// planning.  The base class supplies the one-arm 39-interface MIT protocol.
class TaskSpaceImpedanceMitController final : public DirectMitControllerBase
{
public:
  TaskSpaceImpedanceMitController() : DirectMitControllerBase(DirectMitMode::IMPEDANCE) {}
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  bool uses_raw_topic() const override {return false;}
  using Action = cho_interfaces::action::TaskSpace;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
  enum class Terminal : std::uint8_t {SUCCEEDED, CANCELED, ABORTED};
  struct Goal {
    std::uint64_t id{0};
    double duration{0.0};
    bool relative{false};
    Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
  };
  struct TerminalEvent {std::uint64_t id{0}; Terminal terminal{Terminal::ABORTED};};

  rclcpp_action::GoalResponse goal_callback(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal);
  rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<GoalHandle> & handle);
  void accepted_callback(const std::shared_ptr<GoalHandle> & handle);
  void non_rt_tick();
  void finish(std::uint64_t id, Terminal terminal);
  void abort_active();
  bool write_task_target(double control_time, double dt, DirectMitTarget & target);
  bool configure_task_model();
  bool task_pose_and_jacobian(
    const std::array<double, 7> & q, pinocchio::SE3 & pose,
    Eigen::Matrix<double, 6, 7> & jacobian);
  bool model_nle(const std::array<double, 7> & q, const std::array<double, 7> & dq,
    std::array<double, 7> & nle);
  static bool finite_pose(const Action::Goal & goal);

  std::array<double, 6> kp_task_{}, kd_task_{}, wrench_limit_{};
  std::array<double, 7> startup_posture_{}, startup_start_{}, startup_kp_{}, startup_kd_{};
  double startup_duration_{0.0};
  double startup_tolerance_{0.05};
  double startup_elapsed_{0.0};
  bool startup_active_{false};
  double lambda_{0.05};
  double max_delta_q_{0.002};
  double max_goal_translation_{0.25};
  double max_goal_rotation_{1.57};
  double max_absolute_radius_{1.2};
  double max_cartesian_velocity_{0.10};
  double max_angular_velocity_{0.80};
  std::string ee_frame_{"openarm_hand_tcp"};
  pinocchio::FrameIndex ee_frame_id_{0};
  // Sized once at configure. The RT kinematics path must not allocate.
  Eigen::MatrixXd full_jacobian_;

  rclcpp_action::Server<Action>::SharedPtr task_server_;
  rclcpp::TimerBase::SharedPtr task_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr task_diagnostics_service_;
  realtime_tools::RealtimeBuffer<Goal> task_goal_buffer_{Goal{}};
  realtime_tools::LockFreeSPSCQueue<TerminalEvent, 8> task_terminal_queue_;
  std::mutex task_handles_mutex_;
  std::unordered_map<std::uint64_t, std::shared_ptr<GoalHandle>> task_handles_;
  std::atomic<std::uint64_t> task_next_id_{1};
  std::atomic<std::uint64_t> task_cancel_id_{0};
  std::atomic<bool> task_ready_{false};
  std::atomic<std::uint64_t> task_public_id_{0};
  std::atomic<double> task_percent_{0.0};
  std::uint64_t task_id_{0}, task_last_started_id_{0};
  bool task_compute_failed_{false};
  bool task_cancelled_{false};
  bool task_capacity_rejected_{false};
  double task_start_time_{0.0};
  pinocchio::SE3 task_start_pose_{pinocchio::SE3::Identity()};
  pinocchio::SE3 task_goal_pose_{pinocchio::SE3::Identity()};
  std::array<double, 7> task_q_ref_{};
  std::array<std::atomic<double>, 6> task_last_error_{};
  std::array<std::atomic<double>, 6> task_peak_wrench_{};
  std::array<std::atomic<double>, 7> task_peak_tau_ff_{};
  std::array<std::atomic<double>, 7> task_q_reference_observed_{};
};
}  // namespace cho_controller_openarm_mit
