#include "fairino_hardware/fairino_hardware_interface.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <exception>
#include <string>
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
            _gripper_position_state = percent_to_joint(pkg.gripper_position);
            _gripper_position_command = _gripper_position_state;
            // Seed the sent percentage as well, so the deadband in write()
            // suppresses the first command instead of re-stating the latched
            // position. Restating it would still be a MoveGripper issued
            // milliseconds after activation - the thing that faults this
            // gripper - for no gain: nothing has asked the jaws to move yet.
            // The first real MoveGripper now waits for a controller to ask.
            _gripper_sent_percent = pkg.gripper_position;
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
    _ptr_robot->StopMotion();//停止机器人
    _ptr_robot->CloseRPC();//销毁实例，连接断开
    _ptr_robot.release();
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "System successfully stopped!");
    return hardware_interface::CallbackReturn::SUCCESS;
}



hardware_interface::return_type FairinoHardwareInterface::read(const rclcpp::Time& time,const rclcpp::Duration& period)
{//从RTDE反馈数据中获取所需的位置，速度和扭矩信息
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
            _gripper_position_state = percent_to_joint(pkg.gripper_position);
        }
    }

  return hardware_interface::return_type::OK;

}

hardware_interface::return_type FairinoHardwareInterface::write(const rclcpp::Time& time,const rclcpp::Duration& period)
{
    // cho patch (A3-gripper): written before the arm's control-mode branch so a
    // grasp is not skipped by a mode this build does not implement. MoveGripper
    // is an xmlrpc round trip and write() runs at 125 Hz, so it is only re-sent
    // when the target moves more than a percent of stroke. Non-blocking, so the
    // control loop is never held waiting on the 485 bus.
    if (_has_gripper && _gripper_online) {
        if (!std::isfinite(_gripper_position_command)) {
            return hardware_interface::return_type::ERROR;
        }
        if (_gripper_retry_countdown > 0) {
            --_gripper_retry_countdown;
        } else {
            const double target_percent = joint_to_percent(_gripper_position_command);
            const bool abandoned = _gripper_failed_percent >= 0.0 &&
                std::abs(target_percent - _gripper_failed_percent) < 1.0;
            if (_gripper_failed_percent >= 0.0 && !abandoned) {
                // A target we have not already given up on: worth a try again.
                _gripper_failed_percent = -1.0;
                _gripper_failed_attempts = 0;
            }
            if (!abandoned &&
                (_gripper_sent_percent < 0.0 ||
                 std::abs(target_percent - _gripper_sent_percent) >= 1.0)) {
                const int returncode = _ptr_robot->MoveGripper(
                    _gripper_index, static_cast<int>(std::lround(target_percent)),
                    _gripper_speed_percent, _gripper_force_percent,
                    30000, 1 /*non-blocking*/, 0 /*parallel jaw*/, 0.0, 0, 0);
                if (returncode != 0) {
                    // Sitting out a second beats hammering the 485 bus at 125 Hz,
                    // which turns one rejected grasp into a permanent fault.
                    _gripper_retry_countdown = GRIPPER_RETRY_CYCLES;
                    if (++_gripper_failed_attempts > GRIPPER_MAX_RETRIES) {
                        _gripper_failed_percent = target_percent;
                        RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                                     "MoveGripper to %ld%% has failed %d times (last code %d); "
                                     "giving up on that target. The gripper is most likely "
                                     "faulted - clear it from the teach pendant. A different "
                                     "target will be attempted again.",
                                     std::lround(target_percent), _gripper_failed_attempts,
                                     returncode);
                    } else {
                        RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"),
                                    "MoveGripper to %ld%% failed with code %d; retry %d of %d "
                                    "in %d cycles", std::lround(target_percent), returncode,
                                    _gripper_failed_attempts, GRIPPER_MAX_RETRIES,
                                    GRIPPER_RETRY_CYCLES);
                    }
                } else {
                    _gripper_sent_percent = target_percent;
                    _gripper_failed_attempts = 0;
                    _gripper_failed_percent = -1.0;
                }
            }
        }
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
