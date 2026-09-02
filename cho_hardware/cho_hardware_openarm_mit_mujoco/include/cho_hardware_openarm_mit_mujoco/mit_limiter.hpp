#pragma once
#include <array>
#include <cstdint>
#include <cho_openarm_mit_core/mit_protocol.hpp>
namespace cho_hardware_openarm_mit_mujoco {
constexpr std::size_t N=7;
struct Tuple {double q,dq,kp,kd,tau;};
struct Limits {std::array<double,N> q_lo,q_hi,dq,kp,kd,kp_slew,kd_slew,safe_kp,safe_kd,tau_ff,tau_ff_slew,tau,tau_slew;std::uint64_t lease_cap;};
class Limiter {
public:
 explicit Limiter(const cho_openarm_mit_core::SafetyProfile & profile, std::size_t actual_rate_hz);
 void reset(const std::array<double,N>& measured);
 bool submit(const std::array<Tuple,N>& requested,std::uint64_t generation,std::uint64_t lease);
 std::array<double,N> update(const std::array<double,N>&q,const std::array<double,N>&dq,double dt);
 void fault(){fault_=true;safe_=true;}
 void request_safe(){safe_=true;capture_safe_position_=true;}
 bool safe()const{return safe_;} std::uint64_t ack()const{return ack_;}
private:
 Limits limits_; std::array<Tuple,N> command_{},applied_{}; std::array<double,N> last_tau_{};
 std::uint64_t ack_{0},lease_{0},age_{0}; bool safe_{true},fault_{false},capture_safe_position_{false};
};}
