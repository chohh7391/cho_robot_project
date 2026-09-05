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
  friend struct TaskSpaceImpedanceMitControllerTestAccess;
  bool uses_raw_topic() const override {return false;}
  bool supports_return_to_zero() const override {return true;}
  using Action = cho_interfaces::action::TaskSpace;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
  using Vector6 = Eigen::Matrix<double, 6, 1>;
  using Vector7 = Eigen::Matrix<double, 7, 1>;
  using Jacobian = Eigen::Matrix<double, 6, 7>;
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
  bool write_cartesian_torque_target(
    const pinocchio::SE3 & desired,
    const Vector6 & desired_twist,
    double dt,
    DirectMitTarget & target,
    Vector6 * pose_error_out = nullptr,
    pinocchio::SE3 * measured_pose_out = nullptr);
  bool latch_idle_pose();
  // Release the commanded Cartesian reference toward the measured pose with a
  // cubic blend over `release_duration`.  This replaces both the former
  // measured-pose snap and the former rate limiter on the feedback torque:
  // the reference moves smoothly, so the Kx wrench never steps.
  bool begin_idle_release();
  void begin_idle_release_from(const pinocchio::SE3 & measured_pose);
  static void sample_pose_trajectory(
    const pinocchio::SE3 & start, const pinocchio::SE3 & goal, double u, double duration,
    pinocchio::SE3 & desired, Vector6 & twist);
  // Constrained seven-axis mass matrix, M^-1 J^T and the regularized
  // operational-space inertia Lambda = (J M^-1 J^T + eps I)^-1, computed once
  // per cycle for inertia weighting and the null-space projector.  Without
  // Lambda the Jacobian transpose distributes the task wrench by moment arm
  // alone, so the light wrist absorbs motion that the heavy elbow should be
  // producing - measured on hardware as joint 7 moving 11.65 deg while joint 4
  // moved 0.17 deg for the same Cartesian goal.
  bool task_dynamics(const Jacobian & jacobian);
  // Dynamically consistent null-space posture torque
  // (I - J^T Jbar^T) M (kp_null (q_ref - q) - kd_null dq), Jbar^T = Lambda J M^-1.
  // The 7-DoF arm under a 6-DoF task has a one-dimensional null space that
  // otherwise carries only actuator damping and no restoring force.
  bool nullspace_posture_torque(
    const Jacobian & jacobian, const std::array<double, 7> & q,
    const std::array<double, 7> & dq, Vector7 & torque) const;
  // Coulomb friction feed-forward, tau = scale * level * tanh(v / eps).
  //
  // The velocity whose SIGN is used decides whether this is safe. Measured dq
  // dithers by +/-0.033 rad/s at standstill (one encoder count at the control
  // rate), so sign(dq) would flip hundreds of times a second and swing tau_ff
  // by +/-level each time - a limit cycle at exactly the amplitude of the
  // friction being cancelled. Two sources are therefore offered:
  //
  //   REFERENCE - sign(dq_des), the J^+ velocity reference this controller
  //     already computes. Pure feed-forward: no measured quantity enters, so
  //     the term cannot self-excite, and it is zero whenever the arm is meant
  //     to be still. This is the default and the right choice under control.
  //   MEASURED - sign(dq). The only option for hand guiding, where the motion
  //     comes from the operator and dq_des is identically zero. It closes a
  //     feedback path, so `eps` must sit well above the dither floor and
  //     `scale` must stay below 1: over-compensation turns friction into
  //     negative damping and the arm walks on its own.
  //
  // tanh rather than a hard sign so the term is continuous through zero.
  //
  // The level is Stribeck-shaped rather than constant:
  //
  //   tau = scale * [Fc + (Fs - Fc) * exp(-(v/vs)^2)] * tanh(v / eps)
  //
  // with Fs = friction_level_ and Fc = friction_kinetic_ratio_ * Fs. Breakaway
  // friction is higher than sliding friction, so a level identified at
  // breakaway over-compensates once the joint is actually moving, and the
  // excess is negative damping - measured on hardware as the arm sliding away
  // when hand-guided harder at scale 0.7 while every stationary check passed.
  // A stationary check cannot see this by construction: tanh(0) is zero, so
  // the failure only exists at speed.
  //
  // The bracket decays from Fs to Fc over vs, keeping the breakaway assist
  // where it matters and settling to sliding friction once moving. Both
  // factors stay within [0, 1], so the term is still bounded by scale * Fs.
  // friction_kinetic_ratio_ = 1 collapses this back to the flat law.
  enum class FrictionVelocity : std::uint8_t {REFERENCE, MEASURED};
  void friction_torque(
    const std::array<double, 7> & dq, const std::array<double, 7> & dq_des,
    Vector7 & torque) const;

  // One-sided spring on measured q inside `joint_limit_margin` of the profile
  // window.  The window itself validates only q_des, which the Cartesian modes
  // fill with measured position, so nothing else guards the physical joint.
  void joint_limit_torque(const std::array<double, 7> & q, Vector7 & torque) const;
  // dq_des = J^+ v_des (damped least squares), clamped to the profile command
  // velocity.  The MIT motor evaluates kd*(dq_des - dq) internally; with
  // dq_des = 0 that term brakes every commanded motion, with this reference it
  // tracks it.  J^+ has no null-space component, so the null space still sees
  // -kd*dq.
  void joint_velocity_reference(
    const Jacobian & jacobian, const Vector6 & twist, std::array<double, 7> & dq_des) const;
  // dq = J^+ (x_des ominus x) (damped least squares), clamped per joint by
  // reference_offset_limit_.  Added to measured q it becomes the MIT q_des, so
  // the drive's own kp closes the Cartesian loop inside its current loop
  // instead of the controller synthesising the same force into tau_ff one CAN
  // cycle later.
  void joint_reference_offset(
    const Jacobian & jacobian, const Vector6 & pose_error, Vector7 & offset) const;
  void clamp_command_positions(ArmCommand & command) const;
  // Rate limit for the model feed-forward (nle) only, tracked from the value
  // actually emitted and bounded by the controller torque_limit so saturation
  // cannot wind the tracker up.  The Cartesian PD, null-space and joint-limit
  // torques are never rate-limited: a rate limiter inside a closed loop adds
  // phase lag proportional to error amplitude and is a limit-cycle source.
  double slew_model_feedforward(std::size_t joint, double desired, double dt);
  bool configure_task_model();
  bool task_pose_and_jacobian(
    const std::array<double, 7> & q, pinocchio::SE3 & pose, Jacobian & jacobian);
  bool model_nle(const std::array<double, 7> & q, const std::array<double, 7> & dq,
    std::array<double, 7> & nle);
  static bool finite_pose(const Action::Goal & goal);

  // kp_task/kd_task are runtime-settable so a gain sweep costs one parameter
  // set instead of a relaunch, which would re-home the arm between every
  // point. wrench_limit_ deliberately stays configure-time: it is the last
  // bound on how hard the arm can push and must not be raisable mid-session.
  std::array<std::atomic<double>, 6> kp_task_, kd_task_;
  std::array<double, 6> wrench_limit_{};
  // Where the impedance is evaluated.  True puts it in the MIT drive: the
  // Cartesian error rides q_des and the drive's fixed kp/kd close the loop at
  // current-loop rate.  False keeps the historical law, which builds
  // J^T (Kx e + Dx edot) into tau_ff and therefore closes the same loop across
  // a 200 Hz controller cycle plus CAN transport - the phase lag that made the
  // arm ring during commissioning.
  //
  // kp/kd are FIXED rather than scheduled from diag(J^T Kx J) per pose.  The
  // profile slews kp at 10/s while the mapped stiffness moves 3.4x across the
  // workspace on joint 1 alone, so a scheduled gain would simply lag its own
  // reference.  The off-diagonal coupling of J^T Kx J is dropped for the same
  // reason it is not scheduled: the drive takes a scalar per joint, and
  // routing the remainder through tau_ff would restore the very path this
  // design exists to leave.
  bool drive_side_impedance_{true};
  // Per-joint bound on that reference offset, and the ONLY bound on the
  // impedance torque.  torque_limit_ clamps the effort field alone; the drive
  // evaluates kp*(q_des - q) internally and adds it downstream of anything
  // this controller can clamp.  With kp fixed, |tau_impedance| <= kp * limit.
  std::array<double, 7> reference_offset_limit_{};
  std::array<double, 7> startup_posture_{}, startup_start_{}, startup_kp_{}, startup_kd_{};
  double startup_duration_{0.0};
  double startup_tolerance_{0.05};
  double startup_elapsed_{0.0};
  bool startup_active_{false};
  std::string ee_frame_{"openarm_hand_tcp"};
  pinocchio::FrameIndex ee_frame_id_{0};
  // Sized once at configure. The RT kinematics path must not allocate.
  Eigen::MatrixXd full_jacobian_;
  // When false the controller keeps the plain Jacobian-transpose law and
  // kp_task/kd_task stay in N/m and N*s/m. When true they become acceleration
  // gains (1/s^2 and 1/s) and the emitted wrench is Lambda * (Kx e + Dx edot),
  // still clamped by max_task_wrench so the force bound is unchanged.
  bool task_inertia_weighting_{false};
  double task_inertia_regularization_{1e-4};
  Eigen::Matrix<double, 7, 7> task_arm_mass_{Eigen::Matrix<double, 7, 7>::Identity()};
  Eigen::Matrix<double, 7, 6> task_mass_inverse_jt_{Eigen::Matrix<double, 7, 6>::Zero()};
  Eigen::Matrix<double, 6, 6> task_lambda_{Eigen::Matrix<double, 6, 6>::Identity()};
  bool task_dynamics_valid_{false};
  double task_velocity_reference_damping_{1e-2};
  bool use_nullspace_posture_{false};
  double kp_null_{10.0};
  double kd_null_{1.0};
  bool nullspace_posture_explicit_{false};
  std::array<double, 7> nullspace_posture_{};
  std::array<double, 7> joint_limit_stiffness_{};
  // Breakaway Coulomb level per joint. All zero disables the term entirely,
  // which is the default: an unidentified friction model must not act.
  // Runtime-settable, because friction_scale alone cannot reach past 1 and
  // that cap is relative to THIS number, not to the real friction. When the
  // declared level is an underestimate - as the initial 3%-of-peak-torque
  // guess turned out to be - the scale saturates while the arm is still
  // under-compensated, and the only remaining handle is the level itself.
  // Still bounded by torque_limit per joint on every write.
  std::array<std::atomic<double>, 7> friction_level_;
  // Runtime-settable so the commissioning ramp (0 -> 0.3 -> 0.5 -> 0.7, with a
  // stationary hold checked at each step) does not cost a relaunch per point.
  std::atomic<double> friction_scale_{0.0};
  double friction_velocity_epsilon_{0.1};
  // Runtime-settable for the same reason as friction_scale: the ratio is found
  // by walking it until the arm stops sliding under a hard push, and a
  // relaunch per point would re-home the arm each time.
  std::atomic<double> friction_kinetic_ratio_{1.0};
  std::atomic<double> friction_stribeck_velocity_{0.15};
  FrictionVelocity friction_velocity_source_{FrictionVelocity::REFERENCE};
  double joint_limit_margin_{0.05};
  double release_duration_{1.0};
  // Scales the model term before it is slewed and emitted. The MIT motor has
  // no gravity model of its own, so tau_ff is the arm's entire gravity
  // support; if the URDF is lighter than the real arm - unmodelled tooling,
  // cabling, an optimistic link mass - the arm sags no matter how correct the
  // control law is. This is the empirical handle for that gap and is bounded
  // well below the point where it could drive the arm upward hard.
  // Atomic and runtime-settable: finding the value at which a hand-placed
  // posture holds is a measurement, and making it need a relaunch per step
  // would turn a five-point sweep into five re-homings of the arm.
  std::atomic<double> gravity_scale_{1.0};
  // Per-joint multiplier on the model term, applied on top of gravity_scale.
  // Negative inverts that joint, which exists to test a suspected axis-sign
  // mismatch between the description and the physical motor: with the sign
  // wrong the model term pushes a joint the way gravity already is, so the
  // joint runs away instead of holding. Runtime-settable so the check does not
  // cost a re-homing of the arm.
  std::array<std::atomic<double>, 7> gravity_joint_scale_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr gravity_scale_callback_;

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
  bool task_capacity_rejected_{false};
  double task_start_time_{0.0};
  pinocchio::SE3 task_start_pose_{pinocchio::SE3::Identity()};
  pinocchio::SE3 task_goal_pose_{pinocchio::SE3::Identity()};
  pinocchio::SE3 idle_pose_{pinocchio::SE3::Identity()};
  bool idle_pose_valid_{false};
  bool idle_release_active_{false};
  double idle_release_elapsed_{0.0};
  pinocchio::SE3 idle_release_start_{pinocchio::SE3::Identity()};
  pinocchio::SE3 idle_release_goal_{pinocchio::SE3::Identity()};
  // Joint reference used only by startup and the return-to-zero gain handoff.
  // Active and idle Cartesian control put the Franka task law in MIT tau_ff
  // with zero joint stiffness, and keep `kd` as actuator-side joint damping
  // for the arm's null space.
  std::array<double, 7> task_q_ref_{};
  std::array<double, 7> task_last_model_feedforward_{};
  std::array<std::atomic<double>, 6> task_last_error_{};
  std::array<std::atomic<double>, 6> task_peak_wrench_{};
  std::array<std::atomic<double>, 7> task_peak_tau_ff_{};
  std::array<std::atomic<double>, 7> task_q_reference_observed_{};
};
}  // namespace cho_controller_openarm_mit
