#ifndef _FR_HARDWARE_INTERFACE_
#define _FR_HARDWARE_INTERFACE_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "visibility_control.h"
#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "libfairino/include/robot.h"


#define CONTROLLER_IP_ADDRESS "192.168.58.2"

namespace fairino_hardware
{

class FairinoHardwareInterface: public hardware_interface::SystemInterface{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FairinoHardwareInterface)

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  //FAIRINO_HARDWARE_PUBLIC
  //hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  
  FAIRINO_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  
  FAIRINO_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
  // hardware_interface::return_type prepare_command_mode_switch(
  //   const std::vector<std::string> & start_interfaces,
  //   const std::vector<std::string> & stop_interfaces) override;
  // hardware_interface::return_type perform_command_mode_switch(
  //   const std::vector<std::string>& start_interfaces,
  //   const std::vector<std::string>& stop_interfaces) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
private:
  static constexpr size_t ARM_DOF = 6;

  // cho patch: report WHY ServoJ was refused, not just the code.
  void log_servoj_failure(int returncode);

  // cho patch (A3-gripper) helpers.
  bool activate_gripper();
  // cho patch (A3-gripper-async): the thread that owns every MoveGripper call.
  // Nothing else may call MoveGripper once it is running.
  void gripper_worker();
  bool open_gripper(ROBOT_STATE_PKG& pkg);
  double percent_to_joint(double percent) const;
  double joint_to_percent(double joint) const;

  double _jnt_position_command[6];
  double _jnt_velocity_command[6];
  double _jnt_torque_command[6];
  double _jnt_position_state[6];
  double _jnt_velocity_state[6];
  double _jnt_torque_state[6];
  int _control_mode;
  std::string _controller_ip = CONTROLLER_IP_ADDRESS;
  // The controller's GLOBAL SPEED override, applied on top of every command.
  //
  // 100 means "execute what you are told", which is the only sane default for a
  // driver: the trajectories reaching it are already scaled, and a second
  // invisible factor underneath turns that into a number nobody chose. Upstream
  // never sets it, so whatever the teach pendant was left on carried into every
  // run -- found at 1%, where the arm executed a few percent of its commanded
  // travel, juddered, and aborted every goal on tracking error.
  int _global_speed_percent = 100;
  std::unique_ptr<FRRobot> _ptr_robot;
  // cho patch (A3-gripper-async): one FRRobot is ONE socket, and the SDK does
  // not serialise access to it -- proven the hard way. MoveGripper blocking for
  // 90 s did not delay ServoJ by a single cycle, so there is no lock inside; and
  // with two threads on that socket the SECOND MoveGripper of a session never
  // returned, parked in a socket read waiting for a reply the 125 Hz arm traffic
  // had already taken. Reproduced three times, either direction, with the jaws
  // idle and 38 s between commands, so it is not preemption.
  //
  // So the socket gets the lock the SDK lacks. Timed, not plain: the control
  // loop waits SDK_LOCK_WAIT_MS and then SKIPS its turn rather than blocking,
  // which keeps a gripper that hangs again to a dropped cycle or two instead of
  // a stopped arm. The worker waits as long as it takes -- it is not RT.
  // Kept, but OFF by default -- see _sdk_serialise. Serialising every SDK call
  // behind this is what the gripper needs and what the arm cannot afford.
  std::mutex _sdk_mutex;
  // Whether to take that lock at all. OFF by default, decided on measurement.
  //
  // The lock is what stops a MoveGripper losing its reply to the arm's 125 Hz
  // traffic and never returning. But with it on, the same recorded trajectory
  // tracked far worse: the arm's following lag went from 0.125 s to 0.281 s, and
  // in three of four runs to 0.5-0.67 s, which is past the 0.1 rad state
  // tolerance and an aborted replay. Off, two of two runs completed at 0.06-0.25 s.
  //
  // It is not the cost of taking it -- try_lock on an uncontended mutex is one
  // atomic exchange, and the counter says not one cycle was ever skipped during
  // a replay. Why its mere presence tracks the difference is not understood, and
  // the sample is small enough that it may not. What is not in doubt is which
  // setting finished the trajectory.
  //
  // The gripper is covered without it by _gripper_call_in_flight: write() sits
  // out the cycles where a MoveGripper is actually on the wire, which is under a
  // percent of a run and only at the boundaries where the tree has the arm
  // standing still anyway.
  bool _sdk_serialise = false;
  // Cycles the control loop gave up its turn. Reported by the worker heartbeat
  // rather than logged from the loop itself.
  std::atomic<unsigned long> _sdk_lock_misses{0};
  // Diagnostic: microseconds the last MoveGripper held the socket, and the worst
  // seen. The whole question of whether one socket can carry both is "how long
  // does the gripper hold it", and nothing so far has measured that.
  std::atomic<long> _gripper_hold_us{0};
  std::atomic<long> _gripper_hold_worst_us{0};
  // ...and the same for the cached RTDE reads the control loop makes, which may
  // not touch that socket at all -- in which case they do not belong in the lock.
  std::atomic<long> _read_us_worst{0};

  // cho patch (A3-gripper): an optional RS485 gripper hanging off the robot
  // controller, not off ros2_control. The controller owns the bus, so the only
  // command path is the SDK's MoveGripper(); this exposes it as one extra
  // ros2_control joint so the shared cho_controller_gripper can drive it the
  // same way it drives a simulated finger. Absent the "gripper_joint" hardware
  // parameter the whole block stays dormant and the arm behaves as before.
  bool _has_gripper = false;
  size_t _gripper_joint_index = 0;
  std::string _gripper_joint_name;
  double _gripper_position_command = 0.0;
  double _gripper_position_state = 0.0;
  // Finger travel, in metres, at the fully open end of the joint's range. The
  // SDK speaks percent-of-stroke, so this is the only scale factor needed.
  double _gripper_joint_at_open = 0.0475;
  // The stroke percentage the controller reports and MoveGripper takes, at each
  // end of the joint's travel. 0% closed / 100% open is what holds up when the
  // Gripper action is exercised and the jaws are watched, which is the only
  // check worth trusting here: the stroke READING taken moments after
  // activation contradicted it, because activation itself moves the jaws and
  // re-establishes the gripper's own stroke reference (see activate_gripper).
  // Endpoints rather than an "invert" flag, so a differently wired gripper is a
  // config change and the pair reads like cho_controller_gripper's own width
  // mapping.
  double _gripper_percent_at_closed = 0.0;
  double _gripper_percent_at_open = 100.0;
  int _gripper_index = 1;
  int _gripper_speed_percent = 50;
  int _gripper_force_percent = 30;
  // SetGripperConfig registration, replayed on activation so the controller
  // knows what hangs off its end-effector bus without a visit to the teach
  // pendant. The SDK's encoding: company 4 is DAHUAN, whose only listed device
  // is 0 - PGI-140, the profile the AG-95 is driven with. softversion is
  // documented as unused; bus is the end-effector port.
  int _gripper_company = 4;
  int _gripper_device = 0;
  int _gripper_softversion = 0;
  int _gripper_bus = 1;
  // Whether to WRITE that registration with SetGripperConfig. Off by default:
  // once the teach pendant has registered the gripper the controller keeps it,
  // so rewriting the end-bus device table on every activation buys nothing.
  // GetGripperConfig on this firmware is no help in deciding either - it
  // reported 0/3/0/0 on one run and 1/3/0/0 on the next with nothing written in
  // between, so it cannot be read back as confirmation of anything. The values
  // above are still parsed, so an operator can apply them deliberately once and
  // turn this back off.
  bool _gripper_apply_config = false;
  // Whether a gripper that will not come up should take the whole bringup with
  // it. Off by default: a failed hardware activation makes ros2_control_node
  // throw and abort, so one accessory on a 485 bus would kill the arm too. When
  // it cannot be brought up the arm runs and _gripper_online stays false, which
  // suppresses every gripper command; the log says why.
  bool _gripper_required = false;
  bool _gripper_online = false;
  // Open the jaws once during on_activate. OFF by default, and not for want of
  // trying: activation moves the jaws on its own - a closed gripper came back
  // open and an open one came back closed - and the stroke it reports
  // immediately afterwards cannot be reconciled with what the jaws are actually
  // doing. Commanding an open on top of that produced a gripper that closed at
  // bringup. Issue a Gripper action after bringup instead; that path is
  // verified. The arm latches its measured pose and commands nothing either.
  bool _gripper_open_on_activate = false;
  // Last percentage handed to MoveGripper, or seeded from the measured stroke
  // at activation so that nothing is commanded until a controller asks for a
  // different opening. Owned by the gripper thread once that is running.
  // Negative means "nothing sent and nothing measured".
  double _gripper_sent_percent = -1.0;
  // The percent of stroke write() last asked for, and the ONLY thing the
  // control loop does for the gripper. std::atomic so the store needs no lock
  // the loop could block on; the static_assert in the .cpp refuses to build if
  // it is ever not lock-free, because a hidden lock here is the whole bug this
  // replaced.
  std::atomic<double> _gripper_target_percent{-1.0};
  std::atomic<bool> _gripper_worker_stop{false};
  // Set by the worker when it leaves its loop, so on_deactivate can wait for it
  // WITHOUT betting the shutdown on a MoveGripper that may never return.
  std::atomic<bool> _gripper_worker_done{false};
  // True while the worker is inside MoveGripper. Diagnostic: it is the
  // difference between "the gripper refused" and "the gripper never answered".
  std::atomic<bool> _gripper_call_in_flight{false};
  // The gripper's own "motion finished" register, republished by read().
  //
  // Diagnostic only. It was briefly used to hold a command back until the
  // previous motion finished -- on the theory that overlapping commands were
  // what hung MoveGripper -- but the hang turned out to be the SDK sharing one
  // socket across threads, which _sdk_mutex fixes. Gating on this register only
  // reintroduced the trap activate_gripper already documents: it reads 0 from
  // power-up until the first MoveGripper completes, so the gate sat out its
  // whole 6 s timeout on the first command of every session and the gripper
  // action aborted before the jaws moved. Measured 2026-09-16.
  std::atomic<int> _gripper_motiondone{1};
  // The stroke read OUT OF BAND at activation, with GetGripperCurPosition rather
  // than the RTDE package. Negative if that call failed.
  std::atomic<double> _gripper_verified_percent{-1.0};
  // Until the RTDE package agrees with that reading, read() publishes the
  // verified value instead of the stream. Re-armed by the worker around every
  // MoveGripper, because the register is knocked to 0 for most of a second
  // after one -- see gripper_worker and read().
  std::atomic<bool> _gripper_state_synced{false};
  static constexpr double GRIPPER_SYNC_TOLERANCE_PERCENT = 5.0;
  // MoveGripper's max_time (documented 0~30000 ms, "maximum wait time").
  //
  // Upstream's 30000, restored after measuring what a smaller value actually
  // does. It is NOT just the SDK's own wait: the ROBOT treats it as the deadline
  // the jaw motion must finish inside. At 200 ms the call did return in 24 ms
  // instead of 2061 -- and the jaws still completed their 1.4 s stroke -- but the
  // controller then raised a gripper motion-timeout fault, after which every
  // MoveGripper came back 73 until the fault was cleared from the pendant.
  //
  // So this cannot be used to shorten how long MoveGripper holds the shared
  // socket. That hold is the jaw motion, and the only way to not pay it on the
  // arm's connection would be a second connection, which this controller
  // refuses. Leave it alone.
  static constexpr int GRIPPER_MAX_TIME_MS = 30000;
  // Set when the worker had to be detached, so teardown leaves its socket alone.
  bool _gripper_stuck_at_shutdown = false;
  // Last gripper registers logged, so read() reports them on CHANGE rather than
  // at 125 Hz. -1 means nothing logged yet.
  int _gripper_last_logged[4] = {-1, -1, -1, -1};
  std::thread _gripper_thread;
  // How much the target must move before it is worth an xmlrpc round trip.
  // Percent of stroke, and MoveGripper takes whole percent anyway.
  static constexpr double GRIPPER_DEADBAND_PERCENT = 1.0;
  // How often the worker looks for a new target. The jaws take about a second
  // to move, so polling this slowly costs nothing measurable and buys a write()
  // that makes no syscall at all -- no mutex to contend, no condition variable
  // to wake.
  static constexpr int GRIPPER_POLL_MS = 20;
  // Backoff after a MoveGripper error before trying again. Re-sending an xmlrpc
  // round trip immediately floods the controller, holds the gripper in
  // ERR_GRIPPER_MOTION and buries the log; a second of backoff also throttles
  // the warning to something readable.
  static constexpr int GRIPPER_RETRY_MS = 1000;
  // ...and after this many rejections the target is abandoned rather than
  // retried for the rest of the session. A faulted gripper rejects everything
  // with 73, and a warning a second until shutdown helps nobody. A different
  // target re-arms it, so an operator who clears the fault on the pendant and
  // issues a fresh grasp is served without a restart.
  // Cycles between ServoJ failure reports. Upstream logs every rejected command,
  // which at 125 Hz buries the run in identical lines while telling you nothing
  // the first one did not.
  static constexpr int SERVOJ_LOG_CYCLES = 125;
  int _servoj_log_countdown = 0;
  // Last command ServoJ actually accepted, so a refusal can report the step and
  // the implied joint velocity that was refused rather than just a code.
  double _jnt_position_sent[6] = {0, 0, 0, 0, 0, 0};
  bool _has_sent_position = false;

  // Whether ServoMoveStart succeeded, so on_deactivate only closes servo mode
  // when it was actually opened. robot.h documents ServoMoveStart/ServoMoveEnd
  // as the bracket ServoJ is meant to be streamed inside; upstream called
  // neither.
  // Atomic: the gripper thread closes and reopens servo mode around each
  // MoveGripper, and on_activate/on_deactivate touch it too.
  std::atomic<bool> _servo_mode_open{false};

  static constexpr int GRIPPER_MAX_RETRIES = 5;
  int _gripper_failed_attempts = 0;
  double _gripper_failed_percent = -1.0;  // negative: nothing abandoned
};

} //end namespace


#endif