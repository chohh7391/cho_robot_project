#include "fairino_hardware/fairino_hardware_interface.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <exception>
#include <string>
#include <thread>
#include <type_traits>

namespace fairino_hardware{

hardware_interface::CallbackReturn FairinoHardwareInterface::on_init(const hardware_interface::HardwareInfo& sysinfo){
    if (hardware_interface::SystemInterface::on_init(sysinfo) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    info_ = sysinfo;//info_是父类中定义的变量

    // cho patch: take the controller IP from the ros2_control <param name="robot_ip">
    // (upstream hardcodes it via CONTROLLER_IP_ADDRESS). Falls back to
    // the #define when the parameter is absent.
    if (info_.hardware_parameters.count("robot_ip")) {
        _controller_ip = info_.hardware_parameters.at("robot_ip");
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                    "Using robot_ip from hardware parameters: %s", _controller_ip.c_str());
    }

    // cho patch (A3-gripper): name the joint that stands for the 485 gripper so
    // the arm checks below can skip it. Its interface set is deliberately
    // narrower (position command + position state) because the controller
    // reports one stroke percentage and no finger velocity.
    if (info_.hardware_parameters.count("gripper_joint")) {
        _gripper_joint_name = info_.hardware_parameters.at("gripper_joint");
    }
    auto read_param = [this](const std::string& key, auto fallback) {
        using T = decltype(fallback);
        if (!info_.hardware_parameters.count(key)) {
            return fallback;
        }
        try {
            if constexpr (std::is_same_v<T, int>) {
                return static_cast<T>(std::stoi(info_.hardware_parameters.at(key)));
            } else {
                return static_cast<T>(std::stod(info_.hardware_parameters.at(key)));
            }
        } catch (const std::exception&) {
            RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"),
                        "gripper parameter '%s' is not a number; using the default",
                        key.c_str());
            return fallback;
        }
    };
    _gripper_index = read_param("gripper_index", 1);
    _gripper_joint_at_open = read_param("gripper_joint_at_open", 0.0475);
    _global_speed_percent = read_param("global_speed_percent", 100);
    _gripper_speed_percent = read_param("gripper_speed_percent", 50);
    _gripper_force_percent = read_param("gripper_force_percent", 30);
    _gripper_company = read_param("gripper_company", 4);
    _gripper_device = read_param("gripper_device", 0);
    _gripper_softversion = read_param("gripper_softversion", 0);
    _gripper_bus = read_param("gripper_bus", 1);
    _gripper_percent_at_closed = read_param("gripper_percent_at_closed", 0.0);
    _gripper_percent_at_open = read_param("gripper_percent_at_open", 100.0);
    auto read_flag = [this](const std::string& key, const bool fallback) {
        if (!info_.hardware_parameters.count(key)) {
            return fallback;
        }
        std::string flag = info_.hardware_parameters.at(key);
        std::transform(flag.begin(), flag.end(), flag.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return flag == "true" || flag == "1" || flag == "yes";
    };
    _gripper_open_on_activate = read_flag("gripper_open_on_activate", false);
    _gripper_apply_config = read_flag("gripper_apply_config", false);
    _gripper_required = read_flag("gripper_required", false);
    _sdk_serialise = read_flag("sdk_serialise", false);

    if (_gripper_joint_at_open <= 0.0) {
        RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                     "gripper_joint_at_open must be positive; got %f", _gripper_joint_at_open);
        return hardware_interface::CallbackReturn::ERROR;
    }
    if (std::abs(_gripper_percent_at_open - _gripper_percent_at_closed) < 1.0) {
        RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                     "gripper_percent_at_closed (%f) and gripper_percent_at_open (%f) must "
                     "differ; they are the two ends of the same stroke.",
                     _gripper_percent_at_closed, _gripper_percent_at_open);
        return hardware_interface::CallbackReturn::ERROR;
    }

    size_t arm_joint_count = 0;
    for (size_t index = 0; index < info_.joints.size(); ++index) {
        const hardware_interface::ComponentInfo& joint = info_.joints[index];

        if (!_gripper_joint_name.empty() && joint.name == _gripper_joint_name) {
            // The gripper joint: one position command, one position state.
            if (joint.command_interfaces.size() != 1 ||
                joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
                RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                             "Gripper joint '%s' needs exactly one position command interface.",
                             joint.name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
            if (joint.state_interfaces.size() != 1 ||
                joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
                RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                             "Gripper joint '%s' needs exactly one position state interface.",
                             joint.name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
            _has_gripper = true;
            _gripper_joint_index = index;
            RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                        "Gripper joint '%s' bound to controller gripper index %d "
                        "(open at %.4f m = %.0f%% of stroke, closed = %.0f%%, speed %d%%, "
                        "force %d%%); registering company %d, device %d, softversion %d, "
                        "bus %d",
                        joint.name.c_str(), _gripper_index, _gripper_joint_at_open,
                        _gripper_percent_at_open, _gripper_percent_at_closed,
                        _gripper_speed_percent, _gripper_force_percent,
                        _gripper_company, _gripper_device, _gripper_softversion,
                        _gripper_bus);
            continue;
        }

        ++arm_joint_count;

        //指令部分
        if (joint.command_interfaces.size() != 1) {//开放servoJ
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                        joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                   "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        // if (joint.command_interfaces[1].name != hardware_interface::HW_IF_EFFORT){//预留，用于关节扭矩直接控制
        //     RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
        //            "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
        //            joint.name.c_str(), joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_EFFORT);
        //     return hardware_interface::CallbackReturn::ERROR;
        // }

        //关节状态部分
        // cho patch (A2-velocity): the cho controllers require position+velocity
        // state, so expect two state interfaces (position, velocity) and wire the
        // velocity in read()/export below.
        if (joint.state_interfaces.size() != 2) {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"), "Joint '%s' has %zu state interfaces. 2 expected.",
                        joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                        "Joint '%s' have %s state interface as first state interface. '%s' expected.", joint.name.c_str(),
                        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                        "Joint '%s' have %s state interface as second state interface. '%s' expected.", joint.name.c_str(),
                        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }

        // if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
        //     RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
        //                 "Joint '%s' have %s state interface as third state interface. '%s' expected.", joint.name.c_str(),
        //                 joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_EFFORT);
        //     return hardware_interface::CallbackReturn::ERROR;
        // }

    }

    // cho patch (A3-gripper): the arm state/command arrays are fixed at six, so
    // an unexpected joint count would silently write past their end.
    if (arm_joint_count != ARM_DOF) {
        RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                     "Expected %zu arm joints (plus an optional gripper joint), found %zu.",
                     ARM_DOF, arm_joint_count);
        return hardware_interface::CallbackReturn::ERROR;
    }
    if (!_gripper_joint_name.empty() && !_has_gripper) {
        RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                     "gripper_joint '%s' is not among the declared joints.",
                     _gripper_joint_name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}//end on_init



std::vector<hardware_interface::StateInterface> FairinoHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  //导出关节相关的状态接口(位置，速度，扭矩)
  // cho patch (A3-gripper): the arm arrays are indexed by arm joint, which is no
  // longer the same as the joint index once a gripper joint is declared.
  size_t arm_index = 0;
  for (size_t i = 0; i < info_.joints.size(); ++i){
    if (_has_gripper && i == _gripper_joint_index) {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_gripper_position_state));
      continue;
    }

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_jnt_position_state[arm_index]));

    // cho patch (A2-velocity): export the velocity state filled by read() from
    // GetActualJointSpeedsDegree. _jnt_velocity_state is a C array, so index with [].
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &_jnt_velocity_state[arm_index]));

    // state_interfaces.emplace_back(hardware_interface::StateInterface(
    //     info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &_jnt_torque_state.at(i)));
    ++arm_index;
  }

  //导出
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FairinoHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  size_t arm_index = 0;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    // cho patch (A3-gripper): see export_state_interfaces for the index split.
    if (_has_gripper && i == _gripper_joint_index) {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_gripper_position_command));
      continue;
    }

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_jnt_position_command[arm_index]));

//     command_interfaces.emplace_back(hardware_interface::CommandInterface(//预留的扭矩控制接口
//         info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &_jnt_torque_command.at(i)));
    ++arm_index;
  }

  return command_interfaces;
}



// cho patch (A3-gripper): confirm the controller has the 485 gripper activated
// and seed the command from its measured stroke. Returns false if the gripper is
// unusable, so activation fails loudly instead of at the first grasp.
// cho patch: a bare "ServoJ error 14" says only that the controller refused the
// command. Everything needed to say WHY rides along in the cached state package,
// and the step that was refused is right here, so report both. Code 14 is the
// generic ERR_EXECUTION_FAILED, which is what an over-fast servo step, a
// collision stop, manual mode and a disabled robot all come back as.
void FairinoHardwareInterface::log_servoj_failure(int returncode)
{
    auto logger = rclcpp::get_logger("FairinoHardwareInterface");

    if (_has_sent_position) {
        double worst = 0.0;
        int worst_joint = 0;
        for (int j = 0; j < 6; ++j) {
            const double step = std::abs(_jnt_position_command[j] - _jnt_position_sent[j]);
            if (step > worst) { worst = step; worst_joint = j; }
        }
        // 8 ms is the ServoJ cmdT this driver streams at.
        RCLCPP_ERROR(logger,
                     "ServoJ refused with code %d. Largest commanded step was j%d: "
                     "%.6f rad in one 8 ms cycle = %.3f rad/s (%.1f deg/s). For scale, "
                     "task_space_ik_controller caps itself at 0.005 rad/cycle = 0.625 rad/s.",
                     returncode, worst_joint + 1, worst, worst / 0.008,
                     worst / 0.008 * 180.0 / M_PI);
    } else {
        RCLCPP_ERROR(logger, "ServoJ refused with code %d on the first command it was given.",
                     returncode);
    }

    ROBOT_STATE_PKG pkg = {};
    if (_ptr_robot->GetRobotRealTimeState(&pkg) != 0) {
        RCLCPP_ERROR(logger, "  robot state could not be read to narrow it down further.");
        return;
    }
    RCLCPP_ERROR(logger,
                 "  robot says: mode=%u (0 auto, 1 manual), state=%u (1 stop, 2 run, 3 pause, "
                 "4 drag), enabled=%d, estop=%u, collision=%u, safety_stop=%u/%u, "
                 "program_state=%u, error main=%d sub=%d.",
                 static_cast<unsigned>(pkg.robot_mode), static_cast<unsigned>(pkg.robot_state),
                 pkg.rbtEnableState, static_cast<unsigned>(pkg.EmergencyStop),
                 static_cast<unsigned>(pkg.collisionState),
                 static_cast<unsigned>(pkg.safety_stop0_state),
                 static_cast<unsigned>(pkg.safety_stop1_state),
                 static_cast<unsigned>(pkg.program_state), pkg.main_code, pkg.sub_code);
    if (pkg.collisionState != 0) {
        RCLCPP_ERROR(logger, "  collision=1: the robot's own collision detection has tripped. "
                     "Clear it and re-enable from the pendant; ServoJ stays refused until then.");
    }
    if (pkg.robot_mode != 0) {
        RCLCPP_ERROR(logger, "  mode=1 (manual): ServoJ streaming needs automatic mode.");
    }
    if (pkg.rbtEnableState == 0) {
        RCLCPP_ERROR(logger, "  the robot is not enabled: no motion command will execute.");
    }
}

// cho patch (A3-gripper): open the jaws once on activation and refresh `pkg`
// with the stroke they came to rest at, so the caller seeds the ros2_control
// command from where the fingers actually are.
//
// The stroke has to be POLLED, not read once: MoveGripper's blocking flag does
// not hold until the 485 gripper has finished, and GetRobotRealTimeState hands
// back a cached snapshot, so an immediate re-read returns the pre-motion
// stroke. Seeding from that latched "closed" into the command interface and
// the first write() drove the opening jaws straight back shut, which looked
// like the open command going the wrong way.
bool FairinoHardwareInterface::open_gripper(ROBOT_STATE_PKG& pkg)
{
    using namespace std::chrono_literals;
    auto logger = rclcpp::get_logger("FairinoHardwareInterface");
    const int target_percent = static_cast<int>(std::lround(_gripper_percent_at_open));
    const errno_t returncode = _ptr_robot->MoveGripper(
        _gripper_index, target_percent, _gripper_speed_percent, _gripper_force_percent,
        10000, 0 /*blocking*/, 0 /*parallel jaw*/, 0.0, 0, 0);
    if (returncode != 0) {
        RCLCPP_ERROR(logger, "Opening gripper %d on activation failed with code %d.",
                     _gripper_index, returncode);
        return false;
    }

    // Stroke feedback is an integer percent, and the jaws stop against their
    // own end of travel a percent or two short, so settle for close enough.
    const int tolerance_percent = 3;
    for (int attempt = 0; attempt < 40; ++attempt) {  // up to 10 s
        rclcpp::sleep_for(250ms);
        if (_ptr_robot->GetRobotRealTimeState(&pkg) != 0) {
            continue;
        }
        if (pkg.gripper_fault != 0) {
            // Not fatal. Opening on activation is a convenience, and taking the
            // whole bringup down for it is worse than starting with the jaws
            // where they are: a failed hardware activation makes
            // ros2_control_node throw and abort, so one 485 hiccup on an
            // accessory would kill the arm too.
            RCLCPP_WARN(logger,
                        "Gripper %d reports fault %u while opening (a 485 timeout shows up "
                        "here); leaving the jaws at %u%% and seeding the command there. "
                        "Clear the fault from the pendant, or set gripper: none to bring the "
                        "arm up without it.",
                        _gripper_index, static_cast<unsigned>(pkg.gripper_fault),
                        static_cast<unsigned>(pkg.gripper_position));
            return true;
        }
        if (std::abs(static_cast<int>(pkg.gripper_position) - target_percent)
            <= tolerance_percent) {
            RCLCPP_INFO(logger, "Gripper %d opened: stroke reads %u%%, joint %.4f m.",
                        _gripper_index, static_cast<unsigned>(pkg.gripper_position),
                        percent_to_joint(pkg.gripper_position));
            return true;
        }
    }

    // Something is in the jaws, or they are stuck. Not a reason to refuse the
    // whole bringup: seed from wherever they actually stopped so the command
    // agrees with reality and nothing fights over it, and say so.
    RCLCPP_WARN(logger,
                "Gripper %d did not reach its open end (%d%% of stroke) within 10 s; it "
                "stopped at %u%% (joint %.4f m) and the command is seeded there. Check the "
                "jaws are clear.",
                _gripper_index, target_percent, static_cast<unsigned>(pkg.gripper_position),
                percent_to_joint(pkg.gripper_position));
    return true;
}

bool FairinoHardwareInterface::activate_gripper()
{
    using namespace std::chrono_literals;
    auto logger = rclcpp::get_logger("FairinoHardwareInterface");

    errno_t returncode = 0;
    // Writing the end-bus device table is opt-in. The pendant's registration is
    // what the controller runs on, and rewriting it per activation cost us a
    // 485 timeout for nothing; see _gripper_apply_config.
    if (_gripper_apply_config) {
        returncode = _ptr_robot->SetGripperConfig(_gripper_company, _gripper_device,
                                                  _gripper_softversion, _gripper_bus);
        if (returncode != 0) {
            RCLCPP_ERROR(logger,
                         "SetGripperConfig(company=%d, device=%d, softversion=%d, bus=%d) failed "
                         "with code %d. Check gripper_config in fr5.config.yaml against the SDK's "
                         "table (company 4 is DAHUAN, whose only device is 0 - PGI-140).",
                         _gripper_company, _gripper_device, _gripper_softversion, _gripper_bus,
                         returncode);
            return false;
        }
        rclcpp::sleep_for(1s);
    }

    int company = -1, device = -1, softversion = -1, bus = -1;
    if (_ptr_robot->GetGripperConfig(&company, &device, &softversion, &bus) == 0) {
        RCLCPP_INFO(logger, "Gripper registration on the controller: company %d, device %d, "
                    "softversion %d, bus %d.", company, device, softversion, bus);
        if (_gripper_apply_config &&
            (company != _gripper_company || device != _gripper_device ||
             softversion != _gripper_softversion || bus != _gripper_bus)) {
            RCLCPP_WARN(logger,
                        "That does not match the %d/%d/%d/%d just written. Either this "
                        "firmware's GetGripperConfig reports something else, or the write did "
                        "not take - both are reasons to leave gripper_apply_config off and let "
                        "the pendant own the registration.",
                        _gripper_company, _gripper_device, _gripper_softversion, _gripper_bus);
        }
    }

    // Reset, then activate. This is the step the earlier read-only state check
    // was missing: MoveGripper against a gripper the SDK never activated
    // returns 73 (ERR_GRIPPER_MOTION) on every cycle, and gripper_active alone
    // does not distinguish that case.
    returncode = _ptr_robot->ActGripper(_gripper_index, 0);
    if (returncode != 0) {
        RCLCPP_ERROR(logger, "ActGripper(%d, reset) failed with code %d.",
                     _gripper_index, returncode);
        return false;
    }
    rclcpp::sleep_for(1500ms);
    returncode = _ptr_robot->ActGripper(_gripper_index, 1);
    if (returncode != 0) {
        RCLCPP_ERROR(logger, "ActGripper(%d, activate) failed with code %d.",
                     _gripper_index, returncode);
        return false;
    }

    // Let activation finish before anything is commanded.
    //
    // Activation MOVES THE JAWS, and not to a position we get to choose: a
    // closed gripper came back open and an open one came back closed, which
    // looks like the reset driving to the opposite end and re-establishing the
    // gripper's own stroke reference there. Nothing in the SDK reports this -
    // ActGripper is documented only as 0-reset / 1-activate - so the stroke
    // percentage read straight afterwards cannot be trusted to mean the same
    // physical opening it meant last run. That is why gripper_open_on_activate
    // defaults off: a MoveGripper sent into this either fights the reset or
    // faults the gripper, and a faulted gripper rejects every MoveGripper with
    // 73 until the pendant clears it.
    //
    // gripper_motiondone is NOT the signal for "settled" and is not gated on
    // here: it latches the completion of the last MoveGripper, so it reads 0
    // from power-up until the first one finishes. Requiring it deadlocked
    // activation on an idle, fault-free gripper. A fixed wait is the honest
    // option this firmware leaves.
    rclcpp::sleep_for(3s);

    ROBOT_STATE_PKG pkg = {};
    for (int attempt = 0; attempt < 20; ++attempt) {
        if (_ptr_robot->GetRobotRealTimeState(&pkg) != 0) {
            rclcpp::sleep_for(250ms);
            continue;
        }
        if (pkg.gripper_fault != 0) {
            RCLCPP_ERROR(logger, "Gripper %d reports fault %u (faulting gripper %u); refusing "
                         "to start. A 485 timeout on the pendant shows up here. Clear it from "
                         "the pendant, or set gripper: none in fr5.config.yaml to bring the arm "
                         "up without the gripper.", _gripper_index,
                         static_cast<unsigned>(pkg.gripper_fault),
                         static_cast<unsigned>(pkg.gripper_fault_id));
            return false;
        }
        if (pkg.gripper_active != 0) {
            RCLCPP_INFO(logger, "Gripper %d active (status 0x%04x, motiondone %u) at %u%% of "
                        "stroke (%.4f m).", _gripper_index,
                        static_cast<unsigned>(pkg.gripper_active),
                        static_cast<unsigned>(pkg.gripper_motiondone),
                        static_cast<unsigned>(pkg.gripper_position),
                        percent_to_joint(pkg.gripper_position));
            if (_gripper_open_on_activate && !open_gripper(pkg)) {
                return false;
            }
            // cho patch: take the stroke from GetGripperCurPosition, not from the
            // RTDE package.
            //
            // The package's gripper_position is wrong for a window after
            // activation -- it reported 0% on a gripper whose jaws were open, and
            // 100% on one whose jaws were shut -- and it only starts tracking
            // once the gripper has actually moved. Not cosmetic: this state
            // interface is what cho_controller_gripper seeds its commanded width
            // from when IT activates, moments later, so a 0% reading on an open
            // gripper makes the controller command a close nobody asked for.
            // Seen on every bringup today.
            //
            // GetGripperCurPosition is an xmlrpc round trip -- far too expensive
            // for read() at 125 Hz -- but this is once, at activation, which is
            // the one place the stream cannot be trusted. Both values are logged
            // so the disagreement stays visible rather than becoming folklore.
            uint16_t stroke_fault = 0;
            uint8_t stroke_percent = 0;
            if (_ptr_robot->GetGripperCurPosition(&stroke_fault, &stroke_percent) == 0 &&
                stroke_fault == 0) {
                _gripper_verified_percent.store(static_cast<double>(stroke_percent),
                                                std::memory_order_relaxed);
                RCLCPP_INFO(logger,
                            "Gripper %d stroke: GetGripperCurPosition says %u%%, the RTDE "
                            "package says %u%%.%s", _gripper_index,
                            static_cast<unsigned>(stroke_percent),
                            static_cast<unsigned>(pkg.gripper_position),
                            std::abs(static_cast<double>(stroke_percent) -
                                     static_cast<double>(pkg.gripper_position))
                                    > GRIPPER_SYNC_TOLERANCE_PERCENT
                                ? " Holding the queried value until the package agrees."
                                : "");
            } else {
                _gripper_verified_percent.store(-1.0, std::memory_order_relaxed);
                RCLCPP_WARN(logger,
                            "GetGripperCurPosition failed (fault %u); falling back to the "
                            "RTDE package's %u%%, which cannot be trusted this soon after "
                            "activation.", static_cast<unsigned>(stroke_fault),
                            static_cast<unsigned>(pkg.gripper_position));
            }
            const double verified = _gripper_verified_percent.load(std::memory_order_relaxed);
            _gripper_state_synced.store(verified < 0.0, std::memory_order_relaxed);
            _gripper_position_state = percent_to_joint(
                verified >= 0.0 ? verified : static_cast<double>(pkg.gripper_position));
            _gripper_position_command = _gripper_position_state;
            // Seed the deadband from the VERIFIED stroke -- and only from it.
            //
            // Both halves of this were wrong before. Seeding from the RTDE
            // package taught the deadband a stroke that reading cannot support,
            // and it then swallowed the first real grasp: the action reported
            // SUCCEEDED in 0.2 s with the jaws untouched, and a trajectory replay
            // lost its pick the same silent way. Seeding nothing (-1) fixed that
            // and broke the other end: the first controller command always went
            // out, so activation issued a MoveGripper restating the position the
            // jaws were already at -- and a MoveGripper is exactly what knocks
            // this gripper's position register to 0 for a while. The controller
            // activates ~200 ms later, seeds its commanded width from that 0,
            // and closes a gripper nobody asked to close. Measured: jaws open,
            // both sources reading 100%, redundant MoveGripper(100%), register
            // 0%, controller commands 0%, jaws shut.
            //
            // GetGripperCurPosition is trustworthy where the package is not, so
            // seeding from it suppresses the redundant command WITHOUT swallowing
            // a real one: the deadband now holds a number that matches the jaws.
            // If that query failed, -1 is the honest fallback -- better one
            // redundant command than a silently ignored grasp.
            _gripper_sent_percent = verified;
            return true;
        }
        rclcpp::sleep_for(250ms);
    }
    RCLCPP_ERROR(logger,
                 "Gripper %d did not report active within 5 s of activation (last status "
                 "0x%04x, motiondone %u, fault 0x%04x). Check that it is wired to "
                 "end-effector port %d and registered on the pendant.",
                 _gripper_index, static_cast<unsigned>(pkg.gripper_active),
                 static_cast<unsigned>(pkg.gripper_motiondone),
                 static_cast<unsigned>(pkg.gripper_fault), _gripper_bus);
    return false;
}

// cho patch (A3-gripper): stroke percentage <-> joint position, through the
// configured endpoints. The span is signed, so a gripper that counts 0% open
// (this one) and one that counts 0% closed both work without a special case.
// cho patch (A3-gripper-async): every MoveGripper call happens HERE, on its own
// thread, and nowhere else.
//
// It used to be called inline in write(), immediately before ServoJ, on the same
// FRRobot object -- one TCP connection to the controller, so the two serialise.
// The comment that stood over it ("Non-blocking, so the control loop is never
// held waiting on the 485 bus") read the SDK's block flag wrong: block=1 means
// the ROBOT does not wait for the jaws, not that the call returns before its
// xmlrpc round trip does.
//
// What that cost, measured from /joint_trajectory_controller/controller_state
// over two replays of a recorded trajectory (60 s and 150 s):
//
//   * The 125 Hz loop stalled 0.06-0.30 s at gripper events AND NOWHERE ELSE.
//     Every other inter-sample gap in either run was 0.019 s -- one missed
//     cycle, noise.
//   * Up to ~38 ServoJ points went unsent, the FR5's servo stream underran, and
//     ALL SIX joints stopped. Feedback still dithered, so the arm really was
//     stationary rather than the reading being stale -- and ServoJ kept
//     returning 0 throughout, so nothing in the log marked any of it.
//   * joint_trajectory_controller indexes its trajectory by wall clock, so the
//     reference ran away from a stationary arm. It resumed 2.5-5.7 s later,
//     already past state tolerance, and the goal aborted.
//
// Three gripper events across the two runs, three stalls, freeze length ordered
// by stall length. The nine non-gripper pauses in the same recordings all
// restarted normally, in 0.30-0.64 s.
//
// A thread is as far as this can go on an FR5. franka_ros2 solves the same
// problem one level deeper -- franka_hardware holds no gripper code at all, and
// franka_gripper opens its OWN franka::Gripper on its own port, so the gripper
// never shares the connection the control loop streams on. That does not
// transfer: every Fairino command, ServoJ and MoveGripper alike, goes through
// the one xmlrpc session on port 8080, and the controller refuses a second one.
// Measured, rather than assumed: a second FRRobot::RPC() to the same controller
// while the first was open returned -2, and took ~17 s per attempt to do it.
// So MoveGripper and ServoJ still serialise on one socket; what this thread
// removes is the control loop WAITING for that.
//
// _gripper_sent_percent, _gripper_failed_attempts and _gripper_failed_percent
// belong to this thread once it starts; write() touches none of them.
void FairinoHardwareInterface::gripper_worker()
{
    const auto logger = rclcpp::get_logger("FairinoHardwareInterface");
    const auto poll = std::chrono::milliseconds(GRIPPER_POLL_MS);
    double last_seen = -999.0;              // diagnostic: last target reported
    int heartbeat = 0;
    // A lock inside the atomic would put the wait back in write(), which is the
    // bug this whole function exists to remove.
    static_assert(std::atomic<double>::is_always_lock_free,
                  "std::atomic<double> must be lock-free: write() runs in the "
                  "control loop and may not block on the gripper");

    while (!_gripper_worker_stop.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(poll);
        if (_gripper_worker_stop.load(std::memory_order_relaxed)) {
            break;
        }
        const double target = _gripper_target_percent.load(std::memory_order_relaxed);
        // cho patch (diagnostic): report what the CONTROLLER is asking for,
        // whether or not it is worth sending. Without this, "the gripper did
        // nothing" cannot be told apart from "nothing ever reached the gripper":
        // a run where gripper_controller reported SUCCEEDED while the command
        // interface never moved looks, from here, exactly like an idle one.
        if (std::abs(target - last_seen) >= 0.5) {
            RCLCPP_INFO(logger, "gripper target -> %.1f%% (last sent %.1f%%, jaws %.1f%%)",
                        target, _gripper_sent_percent,
                        joint_to_percent(_gripper_position_state));
            last_seen = target;
        }
        if (++heartbeat >= 5000 / GRIPPER_POLL_MS) {
            heartbeat = 0;
            RCLCPP_INFO(logger,
                        "gripper worker alive: target %.1f%%, sent %.1f%%, jaws %.1f%%, "
                        "cycles yielded %lu, worst MoveGripper hold %.1f ms, "
                        "worst read() %.2f ms",
                        target, _gripper_sent_percent,
                        joint_to_percent(_gripper_position_state),
                        _sdk_lock_misses.load(std::memory_order_relaxed),
                        _gripper_hold_worst_us.load(std::memory_order_relaxed) / 1000.0,
                        _read_us_worst.load(std::memory_order_relaxed) / 1000.0);
        }
        if (!std::isfinite(target) || target < 0.0) {
            continue;                       // nothing has asked for an opening yet
        }
        if (_gripper_failed_percent >= 0.0) {
            if (std::abs(target - _gripper_failed_percent) < GRIPPER_DEADBAND_PERCENT) {
                continue;                   // still the target we gave up on
            }
            // A different one: an operator may have cleared the fault.
            _gripper_failed_percent = -1.0;
            _gripper_failed_attempts = 0;
        }
        if (_gripper_sent_percent >= 0.0 &&
            std::abs(target - _gripper_sent_percent) < GRIPPER_DEADBAND_PERCENT) {
            continue;
        }

        // cho patch: bracket the gripper command the way the vendor brackets
        // ServoJ itself.
        //
        // ServoMoveStart/ServoMoveEnd exist to say when a servo stream is
        // running. Issuing a MoveGripper into an open one is what loses its
        // reply to the arm's 125 Hz traffic -- measured, the second MoveGripper
        // of a session then never returned at all. Closing servo mode first says
        // that plainly to the controller instead of starving the stream behind
        // its back, and reopening after puts it back.
        //
        // in_flight goes up FIRST so write() stops sending ServoJ before servo
        // mode closes under it; the arm holds where it is, which is where the
        // tree has it standing anyway at a gripper event.
        _gripper_call_in_flight.store(true, std::memory_order_relaxed);
        bool reopen_servo = false;
        if (_servo_mode_open.load(std::memory_order_relaxed)) {
            const errno_t endcode = _ptr_robot->ServoMoveEnd();
            if (endcode == 0) {
                _servo_mode_open.store(false, std::memory_order_relaxed);
                reopen_servo = true;
            } else {
                RCLCPP_WARN(logger,
                            "ServoMoveEnd before a gripper command failed with code %d; "
                            "sending it into an open servo stream anyway.", endcode);
            }
        }
        int returncode;
        {
            // Held only for the call. The worker blocks for it -- it is not RT --
            // while the control loop gives up its turn instead.
            std::unique_lock<std::mutex> sdk_guard(_sdk_mutex, std::defer_lock);
            if (_sdk_serialise) {
                sdk_guard.lock();
            }
            const auto held_t0 = std::chrono::steady_clock::now();
            returncode = _ptr_robot->MoveGripper(
                _gripper_index, static_cast<int>(std::lround(target)),
                _gripper_speed_percent, _gripper_force_percent,
                GRIPPER_MAX_TIME_MS,
                // block = 0. Read the MANUAL, not the header's shorthand.
                //
                // robot.h labels this "0-阻塞, 1-非阻塞" and the vendor's own ROS
                // wrapper defaults to 1, so upstream (and this driver until now)
                // sent 1 believing it meant "do not wait". The user manual says
                // the opposite about what these choices DO: "Blocking means that
                // the gripper moves in parallel with the previous movement
                // command." Parallel is the whole point here, and it is 0.
                //
                // The measurement agrees with the manual rather than the header:
                // at 1 the call held the shared socket for exactly as long as the
                // jaws took to move -- 13 ms for a target already reached, 707 ms
                // for a part stroke, 2061 ms for a full one -- which is sequential
                // execution, not "non-blocking".
                0, 0 /*parallel jaw*/,
                0.0, 0, 0);
            const long us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - held_t0).count();
            _gripper_hold_us.store(us, std::memory_order_relaxed);
            if (us > _gripper_hold_worst_us.load(std::memory_order_relaxed)) {
                _gripper_hold_worst_us.store(us, std::memory_order_relaxed);
            }
        }
        if (reopen_servo) {
            const errno_t startcode = _ptr_robot->ServoMoveStart();
            if (startcode == 0) {
                _servo_mode_open.store(true, std::memory_order_relaxed);
            } else {
                RCLCPP_ERROR(logger,
                             "ServoMoveStart after a gripper command failed with code %d. "
                             "The arm is out of servo mode until the next bringup; expect "
                             "the commanded path to lag.", startcode);
            }
        }
        _gripper_call_in_flight.store(false, std::memory_order_relaxed);
        // cho patch: say that a command went out, and what the jaws read when
        // it did. Without this a run where the gripper never moved is
        // indistinguishable from one where it was never commanded -- which is
        // exactly the ambiguity that wasted a 140 s replay. Gripper events are
        // rare, so this costs a handful of lines per run rather than a flood.
        RCLCPP_INFO(logger,
                    "MoveGripper -> %ld%% (code %d) held the socket %.1f ms; jaws read %.1f%%",
                    std::lround(target), returncode,
                    _gripper_hold_us.load(std::memory_order_relaxed) / 1000.0,
                    joint_to_percent(_gripper_position_state));
        if (returncode == 0) {
            // cho patch: distrust the stroke stream again until it comes back.
            //
            // For most of a second after a MoveGripper this gripper reports 0%
            // regardless of where the jaws are, then starts telling the truth.
            // Publishing that 0 is not harmless: it IS the closed target, so a
            // close looked ALREADY ACHIEVED -- cho_controller_gripper saw
            // at_target on its first cycle and finished the grasp in 0.2 s while
            // the jaws had 2.6 s of travel left. The behaviour tree then started
            // the next trajectory on top of a gripper still moving, the robot
            // would not run both, and the arm stood still while its reference ran
            // away; the step that landed when it resumed tripped an axis speed
            // limit. Measured on the replay, 5.7 s of frozen arm.
            //
            // Holding the pre-command stroke until the register agrees with it
            // again costs nothing -- the jaws have not moved yet -- and the
            // completion test then waits for real travel.
            _gripper_verified_percent.store(joint_to_percent(_gripper_position_state),
                                            std::memory_order_relaxed);
            _gripper_state_synced.store(false, std::memory_order_relaxed);
            _gripper_sent_percent = target;
            _gripper_failed_attempts = 0;
            continue;
        }

        if (++_gripper_failed_attempts > GRIPPER_MAX_RETRIES) {
            _gripper_failed_percent = target;
            RCLCPP_ERROR(logger,
                         "MoveGripper to %ld%% has failed %d times (last code %d); giving "
                         "up on that target. The gripper is most likely faulted - clear it "
                         "from the teach pendant. A different target will be attempted "
                         "again.",
                         std::lround(target), _gripper_failed_attempts, returncode);
        } else {
            RCLCPP_WARN(logger,
                        "MoveGripper to %ld%% failed with code %d; retry %d of %d in %d ms",
                        std::lround(target), returncode, _gripper_failed_attempts,
                        GRIPPER_MAX_RETRIES, GRIPPER_RETRY_MS);
        }
        // Backed off in poll-sized steps so a shutdown does not wait out the
        // whole second.
        for (int slept = 0; slept < GRIPPER_RETRY_MS &&
                            !_gripper_worker_stop.load(std::memory_order_relaxed);
             slept += GRIPPER_POLL_MS) {
            std::this_thread::sleep_for(poll);
        }
    }
    _gripper_worker_done.store(true, std::memory_order_release);
}


double FairinoHardwareInterface::percent_to_joint(double percent) const
{
    const double low = std::min(_gripper_percent_at_closed, _gripper_percent_at_open);
    const double high = std::max(_gripper_percent_at_closed, _gripper_percent_at_open);
    const double span = _gripper_percent_at_open - _gripper_percent_at_closed;
    const double fraction = (std::clamp(percent, low, high) - _gripper_percent_at_closed) / span;
    // std::max also normalises the negative zero a reversed span produces at
    // the closed end, which would otherwise show up as -0 in /joint_states.
    return std::max(0.0, std::clamp(fraction, 0.0, 1.0) * _gripper_joint_at_open);
}

double FairinoHardwareInterface::joint_to_percent(double joint) const
{
    const double fraction = std::clamp(joint / _gripper_joint_at_open, 0.0, 1.0);
    return _gripper_percent_at_closed +
           fraction * (_gripper_percent_at_open - _gripper_percent_at_closed);
}

hardware_interface::CallbackReturn FairinoHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
    using namespace std::chrono_literals;
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Starting ...please wait...");
    //做变量的初始化工作
    for(int i=0;i<6;i++){//初始化变量
        _jnt_position_command[i] = 0;
        _jnt_velocity_command[i] = 0;
        _jnt_torque_command[i] = 0;
        _jnt_position_state[i] = 0;
        _jnt_velocity_state[i] = 0;
        _jnt_torque_state[i] = 0;
    }
    _control_mode = 0;//默认是位置控制,0-位置控制，1-扭矩控制 2-速度控制
    // cho patch: retry the xmlrpc connect instead of giving up on one attempt.
    // The controller keeps a session open when a previous ros2_control_node
    // exited without CloseRPC() - an abort skips on_deactivate entirely - and
    // refuses the next connect until that session times out. Upstream tries
    // once, so the first bringup after a crash always failed and the operator
    // had to run it again; the vendor's "check whether the port is occupied"
    // does not point anywhere useful.
    errno_t returncode = -1;
    const int connect_attempts = 4;
    for (int attempt = 1; attempt <= connect_attempts; ++attempt) {
        _ptr_robot = std::make_unique<FRRobot>();
        returncode = _ptr_robot->RPC(_controller_ip.c_str());//建立xmlrpc连接
        rclcpp::sleep_for(200ms);//等待一段时间让控制器的rpc连接建立完毕
        if (returncode == 0) {
            RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "机械臂SDK连接成功！");
            break;
        }
        if (attempt < connect_attempts) {
            RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"),
                        "RPC(%s) failed with code %d on attempt %d of %d; retrying in 3 s. A "
                        "session left open by a previous run clears on its own.",
                        _controller_ip.c_str(), returncode, attempt, connect_attempts);
            rclcpp::sleep_for(3s);
        }
    }
    if (returncode != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                     "机械臂SDK连接失败！RPC(%s) failed with code %d after %d attempts. Check "
                     "the IP and that the robot is reachable; if a previous run was killed "
                     "rather than shut down, its session may still be held - wait and retry.",
                     _controller_ip.c_str(), returncode, connect_attempts);
        return hardware_interface::CallbackReturn::ERROR;
    }
    // cho patch: clear the error state before doing anything else.
    //
    // Unconditional on purpose. The controller latches faults -- an axis speed
    // limit, a gripper motion timeout -- and refuses ServoJ until they are
    // cleared; upstream never calls ResetAllError, so a fault survived every
    // bringup and the only way back was a power cycle of the robot. Nor can the
    // call be made conditional on GetRobotErrorCode: that returned 0/0 on a
    // robot whose own RTDE package was reporting main=1 sub=24 at the same
    // moment, so the two disagree and only one of them is worth trusting to say
    // "nothing to do".
    //
    // Reported before and after rather than done quietly: an error that comes
    // straight back is a real fault, and hiding that would be worse than the
    // power cycle this replaces.
    {
        int before_main = 0, before_sub = 0, after_main = 0, after_sub = 0;
        _ptr_robot->GetRobotErrorCode(&before_main, &before_sub);
        const errno_t resetcode = _ptr_robot->ResetAllError();
        _ptr_robot->GetRobotErrorCode(&after_main, &after_sub);
        if (after_main == 0 && after_sub == 0) {
            RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                        "ResetAllError: code %d, robot error main=%d sub=%d -> %d/%d.",
                        resetcode, before_main, before_sub, after_main, after_sub);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                         "ResetAllError returned %d and the robot STILL reports main=%d "
                         "sub=%d. This one does not clear from software -- check the teach "
                         "pendant, and expect ServoJ to be refused until it does.",
                         resetcode, after_main, after_sub);
        }
    }

    // cho patch: say what the robot IS before using it -- and put it back in
    // AUTO if it is not.
    //
    // Upstream reports the robot's mode only when a ServoJ is refused, which is
    // far too late. A robot left in MANUAL mode accepts the stream and executes
    // a few percent of it: measured, all six joints moved 4-8% of their
    // commanded travel while ServoJ kept returning 0, so the arm juddered, the
    // tracking error ran away and the goal aborted on state tolerance with
    // nothing in any log to say why. Several runs were lost to that before the
    // mode was spotted in a line printed for another reason.
    //
    // Mode(0) rather than only complaining, because on this rig the pendant
    // could not switch it back.
    {
        ROBOT_STATE_PKG pkg = {};
        if (_ptr_robot->GetRobotRealTimeState(&pkg) == 0) {
            RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                        "Robot: mode=%u (%s), state=%u (1 stop, 2 run, 3 pause, 4 drag).",
                        static_cast<unsigned>(pkg.robot_mode),
                        pkg.robot_mode != 0 ? "MANUAL" : "auto",
                        static_cast<unsigned>(pkg.robot_state));
            if (pkg.robot_mode != 0) {
                RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"),
                            "The robot is in MANUAL mode, where it executes only a fraction "
                            "of what it is commanded. Switching to AUTO.");
                const errno_t modecode = _ptr_robot->Mode(0);
                rclcpp::sleep_for(500ms);
                ROBOT_STATE_PKG after = {};
                _ptr_robot->GetRobotRealTimeState(&after);
                if (modecode == 0 && after.robot_mode == 0) {
                    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                                "Robot is now in AUTO mode.");
                } else {
                    RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                                 "Mode(0) returned %d and the robot still reports mode=%u. "
                                 "Set AUTO on the teach pendant before running anything: in "
                                 "MANUAL the arm will judder and every goal will abort.",
                                 modecode, static_cast<unsigned>(after.robot_mode));
                }
            }
        }
    }

    // cho patch: set the global speed override explicitly.
    //
    // It multiplies everything the robot executes and it is NOT in the state
    // package, so it cannot be read back -- it can only be set. Leaving it alone
    // means inheriting whatever the pendant last had, which is how a run ended
    // up executing at 1%: the arm moved 4-8% of its commanded travel on all six
    // joints, ServoJ returned 0 throughout, and every goal aborted on tracking
    // error with no log line naming the cause.
    {
        const errno_t speedcode = _ptr_robot->SetSpeed(_global_speed_percent);
        if (speedcode == 0) {
            RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                        "Global speed override set to %d%%.", _global_speed_percent);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                         "SetSpeed(%d) failed with code %d. The robot keeps whatever the "
                         "pendant last set, which scales every commanded motion -- check it "
                         "there before trusting any tracking number.",
                         _global_speed_percent, speedcode);
        }
    }

    //做第一步的工作，读取当前状态数据
    JointPos jntpos;
    returncode = _ptr_robot->GetActualJointPosDegree(0,&jntpos);
    /*
    获取反馈位置后同步到指令位置以维持当前状态，如果发现读取失败，那么就无法激活插件，
    因为错误的反馈位置会导致初始指令位置下发出现严重偏差导致事故
    */
    if(returncode == 0){
        for(int j=0;j<6;j++){
            _jnt_position_command[j] = jntpos.jPos[j]/180.0*M_PI;
        }
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),"初始指令位置: %f,%f,%f,%f,%f,%f",_jnt_position_command[0],\
        _jnt_position_command[1],_jnt_position_command[2],_jnt_position_command[3],_jnt_position_command[4],_jnt_position_command[5]);    
        // cho patch (A3-gripper): the arm latches its measured pose above so
        // activation commands no motion; do the same for the gripper. Refuse to
        // start if the controller has not activated the gripper, because
        // MoveGripper on an inactive gripper fails silently per cycle.
        if (_has_gripper) {
            _gripper_online = activate_gripper();
            if (!_gripper_online) {
                if (_gripper_required) {
                    return hardware_interface::CallbackReturn::ERROR;
                }
                // Refusing activation makes ros2_control_node throw and abort,
                // taking the arm down with the gripper. Run the arm instead and
                // suppress gripper commands, loudly.
                RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                             "Continuing WITHOUT the gripper: the arm is fully usable and "
                             "gripper commands are ignored until the next bringup. Set "
                             "gripper_required to refuse to start instead, or gripper: none "
                             "in fr5.config.yaml to leave it out of the description.");
            }
            if (_gripper_online) {
                // cho patch (A3-gripper-async): seed the target from what
                // activation latched, so the worker finds nothing to do until a
                // controller asks, then start it. Started here rather than in
                // on_init because it calls MoveGripper on _ptr_robot, and
                // because activate_gripper() must have the bus to itself.
                _gripper_target_percent.store(_gripper_sent_percent,
                                              std::memory_order_relaxed);
                _gripper_worker_stop.store(false, std::memory_order_relaxed);
                _gripper_thread =
                    std::thread(&FairinoHardwareInterface::gripper_worker, this);
            }
        }
        // cho patch: enter servo mode before any ServoJ is streamed.
        //
        // robot.h documents ServoMoveStart/ServoMoveEnd as "配合ServoJ、ServoCart
        // 指令使用" -- to be used TOGETHER WITH ServoJ -- and upstream never
        // calls either, so every ServoJ so far was streamed at a controller that
        // was not in servo mode. Measured on the FR5 with the trajectory
        // controller: the arm trails the commanded path by a lag that GROWS with
        // the session (0.071 s at 25 s, 0.199 s at 150 s, identical on all six
        // joints, so it is the command pipeline and not a joint), and a run that
        // accumulates it faster aborts on state tolerance partway through. That
        // is the shape of commands queueing rather than streaming.
        //
        // It is seeded AFTER the measured pose is latched into
        // _jnt_position_command above, so the first ServoJ that follows asks for
        // where the arm already is.
        //
        // A failure here is logged and NOT fatal: without servo mode the driver
        // behaves exactly as it did before this patch, which is the status quo
        // the rig has been running on. Refusing to activate would turn a
        // degraded arm into no arm.
        const errno_t servocode = _ptr_robot->ServoMoveStart();
        if (servocode == 0) {
            _servo_mode_open.store(true, std::memory_order_relaxed);
            RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                        "Servo mode entered (ServoMoveStart).");
        } else {
            _servo_mode_open.store(false, std::memory_order_relaxed);
            RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                         "ServoMoveStart failed with code %d. Streaming ServoJ anyway, as "
                         "this driver did before servo mode was entered at all - expect the "
                         "commanded path to lag by an amount that grows with the session.",
                         servocode);
        }

        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "机械臂硬件启动成功!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }else{
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "读取初始关节角度错误，硬件无法启动！请检查通讯内容");
        return hardware_interface::CallbackReturn::ERROR;
    }
}



hardware_interface::CallbackReturn FairinoHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Stopping ...please wait...");
    // cho patch (A3-gripper-async): join the gripper thread FIRST. It calls
    // MoveGripper on _ptr_robot, which CloseRPC below tears down.
    if (_gripper_thread.joinable()) {
        _gripper_worker_stop.store(true, std::memory_order_relaxed);
        // Wait, but do NOT bet the shutdown on MoveGripper returning. Measured
        // on this rig: a MoveGripper to a gripper that accepts the command and
        // then never moves does not come back at all -- past its own 30 s
        // maxtime, past 90 s, never. A plain join() here would hang the node on
        // Ctrl-C and leave a session the controller has to time out.
        const int wait_ms = 2000;
        for (int slept = 0; slept < wait_ms &&
                            !_gripper_worker_done.load(std::memory_order_acquire);
             slept += GRIPPER_POLL_MS) {
            rclcpp::sleep_for(std::chrono::milliseconds(GRIPPER_POLL_MS));
        }
        if (_gripper_worker_done.load(std::memory_order_acquire)) {
            _gripper_thread.join();
        } else {
            // Detached, and the connection is deliberately NOT closed below: the
            // thread is parked inside the SDK on that socket, and pulling it out
            // from under it is a crash rather than a clean exit. The process is
            // going away, so leaking both is the cheap option; the controller
            // times the session out on its own (see the connect retry in
            // on_activate).
            RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"),
                        "The gripper thread is still inside MoveGripper after %d ms; "
                        "detaching it and leaving the RPC session to time out. The "
                        "gripper accepted a command and never answered -- check it on "
                        "the teach pendant.", wait_ms);
            _gripper_thread.detach();
            _gripper_stuck_at_shutdown = true;
        }
    }
    // cho patch: close servo mode before stopping, mirroring ServoMoveStart in
    // on_activate. Best effort on purpose -- a shutdown that refuses to finish
    // because the robot would not leave servo mode is worse than one that says
    // so and carries on to StopMotion/CloseRPC. An aborted node skips this
    // callback entirely (see the retry comment in on_activate), so the next
    // activation has to tolerate servo mode already being open.
    if (_servo_mode_open.load(std::memory_order_relaxed)) {
        const errno_t servocode = _ptr_robot->ServoMoveEnd();
        if (servocode != 0) {
            RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"),
                        "ServoMoveEnd failed with code %d; stopping anyway.", servocode);
        }
        _servo_mode_open.store(false, std::memory_order_relaxed);
    }
    _ptr_robot->StopMotion();//停止机器人
    if (!_gripper_stuck_at_shutdown) {
        _ptr_robot->CloseRPC();//销毁实例，连接断开
    }
    _ptr_robot.release();
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "System successfully stopped!");
    return hardware_interface::CallbackReturn::SUCCESS;
}



hardware_interface::return_type FairinoHardwareInterface::read(const rclcpp::Time& time,const rclcpp::Duration& period)
{//从RTDE反馈数据中获取所需的位置，速度和扭矩信息
    // cho patch (A3-gripper-async): the SDK socket is shared with the gripper
    // thread and is not thread-safe; see _sdk_mutex. Missing a read costs
    // nothing -- every branch below already holds its last value on failure.
    std::unique_lock<std::mutex> sdk_guard(_sdk_mutex, std::defer_lock);
    if (_sdk_serialise && !sdk_guard.try_lock()) {
        _sdk_lock_misses.fetch_add(1, std::memory_order_relaxed);
        return hardware_interface::return_type::OK;
    }
    const auto read_t0 = std::chrono::steady_clock::now();
    JointPos state_data;
    error_t returncode = _ptr_robot->GetActualJointPosDegree(1,&state_data);
    if(returncode == 0){
        for(int i=0;i<6;i++){
            _jnt_position_state[i] = state_data.jPos[i]/180.0*M_PI;//注意单位转换，moveit统一用弧度
            //_jnt_torque_state[i] = state_data.jt_cur_tor[i];//注意单位转换
        }
    }else{
        hardware_interface::return_type::ERROR;
    }
    // cho patch (A2-velocity): fill the velocity state (deg/s -> rad/s). Leave the
    // last-known value in place if the RTDE read fails, rather than zeroing it.
    float joint_speed_deg[6];
    if(_ptr_robot->GetActualJointSpeedsDegree(1, joint_speed_deg) == 0){
        for(int i=0;i<6;i++){
            _jnt_velocity_state[i] = joint_speed_deg[i]/180.0*M_PI;
        }
    }
    //RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "System successfully read: %f,%f,%f,%f,%f,%f",_jnt_position_state[0],\
    _jnt_position_state[1],_jnt_position_state[2],_jnt_position_state[3],_jnt_position_state[4],_jnt_position_state[5]);

    // cho patch (A3-gripper): the stroke percentage already rides along in the
    // cached RTDE package, so this costs a snapshot copy rather than another
    // round trip. Hold the last value if the read fails, as the arm state does.
    if (_has_gripper && _gripper_online) {
        ROBOT_STATE_PKG pkg;
        if (_ptr_robot->GetRobotRealTimeState(&pkg) == 0) {
            // cho patch: hold the stroke verified at activation until this
            // stream earns its place -- it agrees within a few percent as soon as
            // it is valid, and it is valid for good once the gripper has been
            // commanded at all, since a real motion is what re-establishes its
            // reference. Publishing it before then is what made the controller
            // seed a closed gripper and command a close.
            const double held = _gripper_verified_percent.load(std::memory_order_relaxed);
            if (!_gripper_state_synced.load(std::memory_order_relaxed) &&
                (held < 0.0 ||
                 std::abs(static_cast<double>(pkg.gripper_position) - held)
                     <= GRIPPER_SYNC_TOLERANCE_PERCENT)) {
                _gripper_state_synced.store(true, std::memory_order_relaxed);
            }
            _gripper_position_state = percent_to_joint(
                _gripper_state_synced.load(std::memory_order_relaxed)
                    ? static_cast<double>(pkg.gripper_position)
                    : held);
            // cho patch (diagnostic): the gripper's own registers, reported when
            // any of them CHANGES rather than at 125 Hz. A gripper that accepts
            // MoveGripper and then never answers says nothing through the return
            // code -- there is no return code yet -- so these are the only view
            // of whether it faulted, is still moving, or simply stopped caring.
            _gripper_motiondone.store(static_cast<int>(pkg.gripper_motiondone),
                                      std::memory_order_relaxed);
            const int now[4] = {
                static_cast<int>(pkg.gripper_active),
                static_cast<int>(pkg.gripper_fault),
                static_cast<int>(pkg.gripper_motiondone),
                static_cast<int>(pkg.gripper_position),
            };
            if (now[0] != _gripper_last_logged[0] || now[1] != _gripper_last_logged[1] ||
                now[2] != _gripper_last_logged[2] || now[3] != _gripper_last_logged[3]) {
                std::copy(now, now + 4, _gripper_last_logged);
                RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                            "gripper regs: active=%d fault=0x%04x motiondone=%d "
                            "position=%d%%  (MoveGripper in flight: %s)",
                            now[0], static_cast<unsigned>(now[1]), now[2], now[3],
                            _gripper_call_in_flight.load(std::memory_order_relaxed)
                                ? "yes" : "no");
            }
        }
    }

    {
        const long us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - read_t0).count();
        if (us > _read_us_worst.load(std::memory_order_relaxed)) {
            _read_us_worst.store(us, std::memory_order_relaxed);
        }
    }
  return hardware_interface::return_type::OK;

}

hardware_interface::return_type FairinoHardwareInterface::write(const rclcpp::Time& time,const rclcpp::Duration& period)
{
    // cho patch (A3-gripper-async): hand the target to the gripper thread and
    // return. One atomic store, no lock and no syscall -- the entire point is
    // that the control loop never waits on the 485 bus. Every MoveGripper call
    // now happens in gripper_worker(), which carries the measurement that made
    // this necessary. Kept before the arm's control-mode branch so a grasp is
    // not skipped by a mode this build does not implement.
    if (_has_gripper && _gripper_online) {
        if (!std::isfinite(_gripper_position_command)) {
            return hardware_interface::return_type::ERROR;
        }
        _gripper_target_percent.store(joint_to_percent(_gripper_position_command),
                                      std::memory_order_relaxed);
    }

    // cho patch (A3-gripper-async): same socket, same lock -- but the control
    // loop never WAITS on the gripper. If the lock is busy this cycle's ServoJ
    // is skipped and the next one carries the newer command; the arm has always
    // absorbed the odd dropped cycle (0.019 s gaps appear throughout every run).
    // Blocking here instead would hand a hung MoveGripper the power to stop the
    // arm, which is the failure this whole change exists to prevent.
    std::unique_lock<std::mutex> sdk_guard(_sdk_mutex, std::defer_lock);
    if (_sdk_serialise && !sdk_guard.try_lock()) {
        _sdk_lock_misses.fetch_add(1, std::memory_order_relaxed);
        return hardware_interface::return_type::OK;
    }
    // cho patch: with the lock off, this is what keeps the gripper's reply from
    // being taken by a ServoJ. One relaxed atomic load, and it is true for the
    // few hundred milliseconds a MoveGripper is actually on the wire -- always
    // at a segment boundary, where the tree is holding the arm still regardless.
    if (!_sdk_serialise && _gripper_call_in_flight.load(std::memory_order_relaxed)) {
        _sdk_lock_misses.fetch_add(1, std::memory_order_relaxed);
        return hardware_interface::return_type::OK;
    }

    if(_control_mode == 0){//位置控制模式
        if (std::any_of(&_jnt_position_command[0], &_jnt_position_command[5],\
            [](double c) { return not std::isfinite(c); })) {
            return hardware_interface::return_type::ERROR;
        }
        JointPos cmd;
        ExaxisPos extcmd{0,0,0,0};
        for(auto j=0;j<6;j++){
            cmd.jPos[j] = _jnt_position_command[j]/M_PI*180; //注意单位转换
        }
        //RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "ServoJ下发位置:%f,%f,%f,%f,%f,%f",\
            cmd.jPos[0],cmd.jPos[1],cmd.jPos[2],cmd.jPos[3],cmd.jPos[4],cmd.jPos[5]);
        int returncode = _ptr_robot->ServoJ(&cmd,&extcmd,0,0,0.008,0,0);
        // cho patch: report once a second, with the refused step and the robot
        // state attached, instead of 125 identical code-only lines a second.
        if (returncode != 0) {
            if (_servoj_log_countdown <= 0) {
                log_servoj_failure(returncode);
                _servoj_log_countdown = SERVOJ_LOG_CYCLES;
            } else {
                --_servoj_log_countdown;
            }
        } else {
            _servoj_log_countdown = 0;
            std::copy(&_jnt_position_command[0], &_jnt_position_command[6],
                      &_jnt_position_sent[0]);
            _has_sent_position = true;
        }
    }else if(_control_mode == 1){//扭矩控制模式
        if (std::any_of(&_jnt_torque_command[0], &_jnt_torque_command[5],\
            [](double c) { return not std::isfinite(c); })) {
            return hardware_interface::return_type::ERROR;
        }
        //_ptr_robot->write(_jnt_torque_command);//注意单位转换
    }else{
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "指令发送错误:未识别当前所处控制模式");
        return hardware_interface::return_type::ERROR;
    }
 
    return hardware_interface::return_type::OK;
}


}//end namesapce

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fairino_hardware::FairinoHardwareInterface, hardware_interface::SystemInterface)
