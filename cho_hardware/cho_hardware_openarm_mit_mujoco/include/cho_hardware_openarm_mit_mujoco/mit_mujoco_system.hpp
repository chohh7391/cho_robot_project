#pragma once
#include <mujoco_ros2_control/mujoco_system_interface.hpp>
#include <memory>
#include "cho_hardware_openarm_mit_mujoco/mit_limiter.hpp"
namespace cho_hardware_openarm_mit_mujoco {
class MitMujocoSystem:public mujoco_ros2_control::MujocoSystemInterface {
public:
 hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo&)override;
 hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&)override;
 hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&)override;
 hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State&)override;
 std::vector<hardware_interface::CommandInterface> export_command_interfaces()override;
 std::vector<hardware_interface::StateInterface> export_state_interfaces()override;
 hardware_interface::return_type read(const rclcpp::Time&,const rclcpp::Duration&)override;
 hardware_interface::return_type write(const rclcpp::Time&,const rclcpp::Duration&)override;
 hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string>&,const std::vector<std::string>&)override;
 hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>&,const std::vector<std::string>&)override;
private:
 void rollback_pending();
 std::vector<std::string> filter_base_claims(const std::vector<std::string>&)const;
 struct Arm {std::string side,resource;std::array<std::string,N> joints{};std::array<std::array<double,5>,N> command{};std::array<double,8> protocol{};double safe_request{0};std::array<hardware_interface::CommandInterface*,N> raw_effort{};std::array<const hardware_interface::StateInterface*,N> position{},velocity{};std::unique_ptr<Limiter> limiter,shadow;std::uint64_t submitted{0},observed{0};bool direct_owned{false};};
 std::vector<Arm> arms_;std::vector<hardware_interface::CommandInterface> base_commands_;std::vector<hardware_interface::StateInterface> base_states_;std::uint64_t next_session_{1};
 bool paired_owned_{false};
 double pair_ownership_token_{0};
 double pair_stop_ready_{0};
 bool pending_pair_{false};bool pending_clear_pair_{false};std::array<bool,2> pending_direct_{false,false};
};}
