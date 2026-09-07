#ifndef _FR_HARDWARE_INTERFACE_
#define _FR_HARDWARE_INTERFACE_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "visibility_control.h"
#include <string>
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
  std::unique_ptr<FRRobot> _ptr_robot;

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
  // different opening. write() runs at 125 Hz and MoveGripper is an xmlrpc
  // round trip, so the command is only sent when the target moves outside a
  // deadband. Negative means "nothing sent and nothing measured".
  double _gripper_sent_percent = -1.0;
  // Cycles to sit out after a MoveGripper error before trying again. Re-sending
  // an xmlrpc round trip every 8 ms floods the controller, holds the gripper in
  // ERR_GRIPPER_MOTION and buries the log; a second of backoff also throttles
  // the warning to something readable.
  static constexpr int GRIPPER_RETRY_CYCLES = 125;
  int _gripper_retry_countdown = 0;
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

  static constexpr int GRIPPER_MAX_RETRIES = 5;
  int _gripper_failed_attempts = 0;
  double _gripper_failed_percent = -1.0;  // negative: nothing abandoned
};

} //end namespace


#endif